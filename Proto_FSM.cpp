#include "unified_controller.h"
#include <math.h>
#include <stdio.h>

#define PI 3.141592
#define RAD2DEG 57.295791433
#define DEG2RAD 0.017453289


CUController::CUController() //calc global position, implement fsm
{
    _now_time4 = 0.0;
    _cnt = 0;
    _control_arm_num = 0;//0 = none, 1:forward arm, 2: backward arm

    _CurrentState = Ready;
    _PreviousState = Ready;
    _CurrentTask = No_Task;
    _init_time = 0.0;
    _operation_time = 0.0;
 
    for(int i=0; i<3; i++)
    {
        _q_goal_forward[i] = 0.0;
        _x_goal_forward[i] = 0.0;
    }
    _height_goal_linear = 0.0;
    _bool_ee_control = false;

}

CUController::~CUController()
{
}

void CUController::Finite_State_Machine( int button, float axes1, float axes2, double time)
{   //std::cout <<time << std::endl;
    //std::cout << _CurrentState << std::endl;
    //std::cout<< button << axes1 << axes2 << std::endl;
    //std::cout << _new_mode << std::endl;

    //TODO: ros joy => what is button, axes1, axes2? Check
    Linear_backward.write_cmd_from_FSM(5.0, 1.0);
    _control_arm_num = 1; //0 = none, 1:forward arm, 2: backward arm    

    if(_CurrentState == Ready)
    {
		_q_goal_forward[0] = -80.0*DEG2RAD; //q0
		_q_goal_forward[1] = 160.0*DEG2RAD; //q1
		_q_goal_forward[2] = 10.0*DEG2RAD; //q2
		_height_goal_linear = 0.57; //z 
        _bool_ee_control = false; //joint control
        _operation_time = 10.0; 

        if( button == 1)
        {
            _CurrentTask = No_Task;
            _CurrentState = Ready;
            _PreviousState = Ready;
        }
        else if ( axes1 < 0 ) //왼쪽버튼 누를시 -> 왼쪽 선반으로 //
        {
            cout << "Test Motion 1" <<endl;
            _CurrentTask = Test_Task;
            _CurrentState = Table_F1_Lane;
            _init_time = time;      
            _PreviousState = Ready;
        }
        else if ( axes2 < 0 ) //오른쪽버튼 누를시 -> 오른쪽 선반으로 //
        {
            _CurrentTask = IK_Task;
            _CurrentState = IK_State;
            _init_time = time;      
            _PreviousState = Ready;
            
            
            /*
            cout << "Test Motion 2" <<endl;
            _CurrentTask = Test_Task;
            _CurrentState = Table_2_Lane;
            _init_time = time;      
            _PreviousState = Ready;
            */
        }
        else 
        {
            _CurrentTask = No_Task;
            _CurrentState = Ready;
            _PreviousState = Ready;
        }

        Scara_forward.write_cmd_from_FSM(_operation_time, _q_goal_forward, _x_goal_forward, _bool_ee_control);
        Linear_forward.write_cmd_from_FSM(_operation_time, _height_goal_linear);
        tray_forward.write_cmd_from_FSM(5.0,0.0);
    }


/////////////////////////////////////////////////////////// IK State /////////////////////////////////////////////////////////////
   
    else if(_CurrentState == IK_State)
    {
        _operation_time = 20.0;
        
        std::cout << "Please enter x,y,z,a values : ";
		std::cin >> IK_x;
		std::cin >> IK_y;
		std::cin >> IK_z;
        std::cin >> IK_a;
		std::cout << "x = " << IK_x << ", " << "y = " << IK_y << ", " << "Scara_forwardz = " << IK_z << ", " << "a = " << IK_a << std::endl; 
        
		Scara_forward.calc_iversekinematics( IK_x, IK_y, IK_a); //solve IK to get _q[1]~[3]
		_q_goal_forward[0] = Scara_forward._x_ee_goal_local[0];
        _q_goal_forward[1] = Scara_forward._x_ee_goal_local[1];
        _q_goal_forward[2] = Scara_forward._x_ee_goal_local[2];
        std::cout << "_q_goal_forward" << std::endl;
        std::cout << _q_goal_forward[0] << std::endl;
        std::cout << _q_goal_forward[1] << std::endl;
        std::cout << _q_goal_forward[2] << std::endl;
        
        Scara_forward.write_cmd_from_FSM(_operation_time, _q_goal_forward, _x_goal_forward, _bool_ee_control);
        Linear_forward.write_cmd_from_FSM(_operation_time, _height_goal_linear);
        //Linear_forward.write_cmd_from_FSM(10, IK_z);
    }

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


     // Table F1 lane
    else if(_CurrentState == Table_F1_Lane)
    {
        _operation_time = 10.0;
        
		if(time < _init_time + _operation_time)
        {
            _q_goal_forward[0] = -15.34*DEG2RAD; //q0
		    _q_goal_forward[1] = 145.65*DEG2RAD; //q1
		    _q_goal_forward[2] = 48.45*DEG2RAD; //q2
		    _height_goal_linear = 1.39 - 0.726; //z 
        }
        else if(time >= _init_time + _operation_time)
        {
            if(_PreviousState == Ready)
            {
                _CurrentState = Table_F1;
                _init_time = time;          
                _PreviousState = Table_F1_Lane;
            }
            else if(_PreviousState == Item_Move)
            {
                _CurrentState = Drop_Lane;
                _init_time = time;          
                _PreviousState = Table_F1_Lane;
            }
        }       

        Scara_forward.write_cmd_from_FSM(_operation_time, _q_goal_forward, _x_goal_forward, _bool_ee_control);
        Linear_forward.write_cmd_from_FSM(_operation_time, _height_goal_linear);
        tray_forward.write_cmd_from_FSM(5.0,0.0);
    }

     // Table F2 lane
    else if(_CurrentState == Table_F2_Lane)
    {
        _operation_time = 10.0;
        if(time < _init_time + _operation_time)
        {
		    _q_goal_forward[0] = 13.83*DEG2RAD; //q0
		    _q_goal_forward[1] = -146.29*DEG2RAD; //q1
		    _q_goal_forward[2] = 132.32*DEG2RAD; //q2
		    _height_goal_linear = 1.39 - 1.046; //z 
        }
        else if(time >= _init_time + _operation_time)
        {
            if(_PreviousState == Ready)
            {
                _CurrentState = Table_F2;
                _init_time = time;          
                _PreviousState = Table_F2_Lane;
            }
            else if(_PreviousState == Item_Move)
            {
                _CurrentState = Drop_Lane;
                _init_time = time;          
                _PreviousState = Table_F2_Lane;
            }
        }        

        Scara_forward.write_cmd_from_FSM(_operation_time, _q_goal_forward, _x_goal_forward, _bool_ee_control);
        Linear_forward.write_cmd_from_FSM(_operation_time, _height_goal_linear);
        tray_forward.write_cmd_from_FSM(5.0,0.0);
    }

     // Table F3 lane
    else if(_CurrentState == Table_F3_Lane)
    {
        _operation_time = 10.0;
        if(time < _init_time + _operation_time)
        {
		    _q_goal_forward[0] = 13.83*DEG2RAD; //q0
		    _q_goal_forward[1] = -146.29*DEG2RAD; //q1
		    _q_goal_forward[2] = 132.32*DEG2RAD; //q2
		    _height_goal_linear = 1.39 - 1.046; //z 
        }
        else if(time >= _init_time + _operation_time)
        {
            if(_PreviousState == Ready)
            {
                _CurrentState = Table_F3;
                _init_time = time;          
                _PreviousState = Table_F3_Lane;
            }
            else if(_PreviousState == Item_Move)
            {
                _CurrentState = Drop_Lane;
                _init_time = time;          
                _PreviousState = Table_F3_Lane;
            }
        }        

        Scara_forward.write_cmd_from_FSM(_operation_time, _q_goal_forward, _x_goal_forward, _bool_ee_control);
        Linear_forward.write_cmd_from_FSM(_operation_time, _height_goal_linear);
        tray_forward.write_cmd_from_FSM(5.0,0.0);
    }

     // Table F4 lane
    else if(_CurrentState == Table_F4_Lane)
    {
        _operation_time = 10.0;
        if(time < _init_time + _operation_time)
        {
		    _q_goal_forward[0] = 13.83*DEG2RAD; //q0
		    _q_goal_forward[1] = -146.29*DEG2RAD; //q1
		    _q_goal_forward[2] = 132.32*DEG2RAD; //q2
		    _height_goal_linear = 1.39 - 1.046; //z 
        }
        else if(time >= _init_time + _operation_time)
        {
            if(_PreviousState == Ready)
            {
                _CurrentState = Table_F4;
                _init_time = time;          
                _PreviousState = Table_F4_Lane;
            }
            else if(_PreviousState == Item_Move)
            {
                _CurrentState = Drop_Lane;
                _init_time = time;          
                _PreviousState = Table_F4_Lane;
            }
        }        

        Scara_forward.write_cmd_from_FSM(_operation_time, _q_goal_forward, _x_goal_forward, _bool_ee_control);
        Linear_forward.write_cmd_from_FSM(_operation_time, _height_goal_linear);
        tray_forward.write_cmd_from_FSM(5.0,0.0);
    }

     // Table F5 lane
    else if(_CurrentState == Table_F5_Lane)
    {
        _operation_time = 10.0;
        if(time < _init_time + _operation_time)
        {
		    _q_goal_forward[0] = 13.83*DEG2RAD; //q0
		    _q_goal_forward[1] = -146.29*DEG2RAD; //q1
		    _q_goal_forward[2] = 132.32*DEG2RAD; //q2
		    _height_goal_linear = 1.39 - 1.046; //z 
        }
        else if(time >= _init_time + _operation_time)
        {
            if(_PreviousState == Ready)
            {
                _CurrentState = Table_F5;
                _init_time = time;          
                _PreviousState = Table_F5_Lane;
            }
            else if(_PreviousState == Item_Move)
            {
                _CurrentState = Drop_Lane;
                _init_time = time;          
                _PreviousState = Table_F5_Lane;
            }
        }        

        Scara_forward.write_cmd_from_FSM(_operation_time, _q_goal_forward, _x_goal_forward, _bool_ee_control);
        Linear_forward.write_cmd_from_FSM(_operation_time, _height_goal_linear);
        tray_forward.write_cmd_from_FSM(5.0,0.0);
    }

     // Table F6 lane
    else if(_CurrentState == Table_F6_Lane)
    {
        _operation_time = 10.0;
        if(time < _init_time + _operation_time)
        {
		    _q_goal_forward[0] = 13.83*DEG2RAD; //q0
		    _q_goal_forward[1] = -146.29*DEG2RAD; //q1
		    _q_goal_forward[2] = 132.32*DEG2RAD; //q2
		    _height_goal_linear = 1.39 - 1.046; //z 
        }
        else if(time >= _init_time + _operation_time)
        {
            if(_PreviousState == Ready)
            {
                _CurrentState = Table_F6;
                _init_time = time;          
                _PreviousState = Table_F6_Lane;
            }
            else if(_PreviousState == Item_Move)
            {
                _CurrentState = Drop_Lane;
                _init_time = time;          
                _PreviousState = Table_F6_Lane;
            }
        }        

        Scara_forward.write_cmd_from_FSM(_operation_time, _q_goal_forward, _x_goal_forward, _bool_ee_control);
        Linear_forward.write_cmd_from_FSM(_operation_time, _height_goal_linear);
        tray_forward.write_cmd_from_FSM(5.0,0.0);
    }

     // Table F7 lane
    else if(_CurrentState == Table_F7_Lane)
    {
        _operation_time = 10.0;
        if(time < _init_time + _operation_time)
        {
		    _q_goal_forward[0] = 13.83*DEG2RAD; //q0
		    _q_goal_forward[1] = -146.29*DEG2RAD; //q1
		    _q_goal_forward[2] = 132.32*DEG2RAD; //q2
		    _height_goal_linear = 1.39 - 1.046; //z 
        }
        else if(time >= _init_time + _operation_time)
        {
            if(_PreviousState == Ready)
            {
                _CurrentState = Table_F7;
                _init_time = time;          
                _PreviousState = Table_F7_Lane;
            }
            else if(_PreviousState == Item_Move)
            {
                _CurrentState = Drop_Lane;
                _init_time = time;          
                _PreviousState = Table_F7_Lane;
            }
        }        

        Scara_forward.write_cmd_from_FSM(_operation_time, _q_goal_forward, _x_goal_forward, _bool_ee_control);
        Linear_forward.write_cmd_from_FSM(_operation_time, _height_goal_linear);
        tray_forward.write_cmd_from_FSM(5.0,0.0);
    }

     // Table F8 lane
    else if(_CurrentState == Table_F8_Lane)
    {
        _operation_time = 10.0;
        if(time < _init_time + _operation_time)
        {
		    _q_goal_forward[0] = 13.83*DEG2RAD; //q0
		    _q_goal_forward[1] = -146.29*DEG2RAD; //q1
		    _q_goal_forward[2] = 132.32*DEG2RAD; //q2
		    _height_goal_linear = 1.39 - 1.046; //z 
        }
        else if(time >= _init_time + _operation_time)
        {
            if(_PreviousState == Ready)
            {
                _CurrentState = Table_F8;
                _init_time = time;          
                _PreviousState = Table_F8_Lane;
            }
            else if(_PreviousState == Item_Move)
            {
                _CurrentState = Drop_Lane;
                _init_time = time;          
                _PreviousState = Table_F8_Lane;
            }
        }        

        Scara_forward.write_cmd_from_FSM(_operation_time, _q_goal_forward, _x_goal_forward, _bool_ee_control);
        Linear_forward.write_cmd_from_FSM(_operation_time, _height_goal_linear);
        tray_forward.write_cmd_from_FSM(5.0,0.0);
    }


    // Table F1
    else if(_CurrentState == Table_F1)
    {
        _operation_time = 5.0; 

        if(time >= _init_time && time < _init_time + _operation_time)
        {
		    _q_goal_forward[0] = -11.84*DEG2RAD; //q0
		    _q_goal_forward[1] = 146.23*DEG2RAD; //q1
		    _q_goal_forward[2] = 45.67*DEG2RAD; //q2
		    _height_goal_linear = 1.39 - 0.726; //z 
        }       
        else if((time >= _init_time + _operation_time) && (time < _init_time + _operation_time + 5.0))
        {
		    _q_goal_forward[0] = -7.53*DEG2RAD; //q0
		    _q_goal_forward[1] = 148.24*DEG2RAD; //q1
		    _q_goal_forward[2] = 38.44*DEG2RAD; //q2
		    _height_goal_linear = 1.39 - 0.726; //z 
        }
        else if(time >= _init_time + _operation_time + 5.0)
        {
            if(_CurrentTask == Test_Task)
            {
                _CurrentState = Item_Move;
                _init_time = time;              
            }
            _PreviousState = Table_F1;
        }   
        Scara_forward.write_cmd_from_FSM(_operation_time, _q_goal_forward, _x_goal_forward, _bool_ee_control);
        Linear_forward.write_cmd_from_FSM(_operation_time, _height_goal_linear);
        tray_forward.write_cmd_from_FSM(5.0,0.0);
    }
    // Table F2
    else if(_CurrentState == Table_F2)
    {
        _operation_time = 5.0;

        if(time >= _init_time && time < _init_time + _operation_time)
        {
		    _q_goal_forward[0] = -11.84*DEG2RAD; //q0
		    _q_goal_forward[1] = 146.23*DEG2RAD; //q1
		    _q_goal_forward[2] = 45.67*DEG2RAD; //q2
		    _height_goal_linear = 1.39 - 0.726; //z 
        }       
        else if(time >= _init_time + _operation_time && time < _init_time + _operation_time + 5.0)
        {
		    _q_goal_forward[0] = -7.22*DEG2RAD; //q0
		    _q_goal_forward[1] = 148.11*DEG2RAD; //q1
		    _q_goal_forward[2] = 39.31*DEG2RAD; //q2
		    _height_goal_linear = 1.39 - 1.046; //z 
        }
        else if(time >= _init_time + _operation_time + 5.0)
        {
            if(_CurrentTask == Test_Task)
            {
                _CurrentState = Item_Move;
                _init_time = time;              
            }
            _PreviousState = Table_F2;
        }   

        Scara_forward.write_cmd_from_FSM(_operation_time, _q_goal_forward, _x_goal_forward, _bool_ee_control);
        Linear_forward.write_cmd_from_FSM(_operation_time, _height_goal_linear);
        tray_forward.write_cmd_from_FSM(5.0,0.0);
    } 

    // Table F3
    else if(_CurrentState == Table_F3)
    {
        _operation_time = 5.0; 

        if(time >= _init_time && time < _init_time + _operation_time)
        {
		    _q_goal_forward[0] = -11.84*DEG2RAD; //q0
		    _q_goal_forward[1] = 146.23*DEG2RAD; //q1
		    _q_goal_forward[2] = 45.67*DEG2RAD; //q2
		    _height_goal_linear = 1.39 - 0.726; //z 
        }       
        else if((time >= _init_time + _operation_time) && (time < _init_time + _operation_time + 5.0))
        {
		    _q_goal_forward[0] = -7.15*DEG2RAD; //q0
		    _q_goal_forward[1] = 148.07*DEG2RAD; //q1
		    _q_goal_forward[2] = 39.18*DEG2RAD; //q2
		    _height_goal_linear = 1.39 - 0.726; //z 
        }
        else if(time >= _init_time + _operation_time + 5.0)
        {
            if(_CurrentTask == Test_Task)
            {
                _CurrentState = Item_Move;
                _init_time = time;              
            }
            _PreviousState = Table_F3;
        }   
        Scara_forward.write_cmd_from_FSM(_operation_time, _q_goal_forward, _x_goal_forward, _bool_ee_control);
        Linear_forward.write_cmd_from_FSM(_operation_time, _height_goal_linear);
        tray_forward.write_cmd_from_FSM(5.0,0.0);
    }

    // Table F4
    else if(_CurrentState == Table_F4)
    {
        _operation_time = 5.0; 

        if(time >= _init_time && time < _init_time + _operation_time)
        {
		    _q_goal_forward[0] = -11.84*DEG2RAD; //q0
		    _q_goal_forward[1] = 146.23*DEG2RAD; //q1
		    _q_goal_forward[2] = 45.67*DEG2RAD; //q2
		    _height_goal_linear = 1.39 - 0.726; //z 
        }       
        else if((time >= _init_time + _operation_time) && (time < _init_time + _operation_time + 5.0))
        {
		    _q_goal_forward[0] = -7.24*DEG2RAD; //q0
		    _q_goal_forward[1] = 148.29*DEG2RAD; //q1
		    _q_goal_forward[2] = 39.23*DEG2RAD; //q2
		    _height_goal_linear = 1.39 - 0.726; //z 
        }
        else if(time >= _init_time + _operation_time + 5.0)
        {
            if(_CurrentTask == Test_Task)
            {
                _CurrentState = Item_Move;
                _init_time = time;              
            }
            _PreviousState = Table_F4;
        }   
        Scara_forward.write_cmd_from_FSM(_operation_time, _q_goal_forward, _x_goal_forward, _bool_ee_control);
        Linear_forward.write_cmd_from_FSM(_operation_time, _height_goal_linear);
        tray_forward.write_cmd_from_FSM(5.0,0.0);
    }

    // Table F5
    else if(_CurrentState == Table_F5)
    {
        _operation_time = 5.0; 

        if(time >= _init_time && time < _init_time + _operation_time)
        {
		    _q_goal_forward[0] = -11.84*DEG2RAD; //q0
		    _q_goal_forward[1] = 146.23*DEG2RAD; //q1
		    _q_goal_forward[2] = 45.67*DEG2RAD; //q2
		    _height_goal_linear = 1.39 - 0.726; //z 
        }       
        else if((time >= _init_time + _operation_time) && (time < _init_time + _operation_time + 5.0))
        {
		    _q_goal_forward[0] = -7.23*DEG2RAD; //q0
		    _q_goal_forward[1] = 148.22*DEG2RAD; //q1
		    _q_goal_forward[2] = 39.15*DEG2RAD; //q2
		    _height_goal_linear = 1.39 - 0.726; //z 
        }
        else if(time >= _init_time + _operation_time + 5.0)
        {
            if(_CurrentTask == Test_Task)
            {
                _CurrentState = Item_Move;
                _init_time = time;              
            }
            _PreviousState = Table_F5;
        }   
        Scara_forward.write_cmd_from_FSM(_operation_time, _q_goal_forward, _x_goal_forward, _bool_ee_control);
        Linear_forward.write_cmd_from_FSM(_operation_time, _height_goal_linear);
        tray_forward.write_cmd_from_FSM(5.0,0.0);
    }

    // Table F6
    else if(_CurrentState == Table_F6)
    {
        _operation_time = 5.0; 

        if(time >= _init_time && time < _init_time + _operation_time)
        {
		    _q_goal_forward[0] = 8.91*DEG2RAD; //q0
		    _q_goal_forward[1] = -147.88*DEG2RAD; //q1
		    _q_goal_forward[2] = 137.98*DEG2RAD; //q2
		    _height_goal_linear = 1.39 - 0.726; //z 
        }       
        else if((time >= _init_time + _operation_time) && (time < _init_time + _operation_time + 5.0))
        {
		    _q_goal_forward[0] = 7.12*DEG2RAD; //q0
		    _q_goal_forward[1] = -148.79*DEG2RAD; //q1
		    _q_goal_forward[2] = 140.84*DEG2RAD; //q2
		    _height_goal_linear = 1.39 - 0.726; //z 
        }
        else if(time >= _init_time + _operation_time + 5.0)
        {
            if(_CurrentTask == Test_Task)
            {
                _CurrentState = Item_Move;
                _init_time = time;              
            }
            _PreviousState = Table_F6;
        }   
        Scara_forward.write_cmd_from_FSM(_operation_time, _q_goal_forward, _x_goal_forward, _bool_ee_control);
        Linear_forward.write_cmd_from_FSM(_operation_time, _height_goal_linear);
        tray_forward.write_cmd_from_FSM(5.0,0.0);
    }

    // Table F7
    else if(_CurrentState == Table_F7)
    {
        _operation_time = 5.0; 

        if(time >= _init_time && time < _init_time + _operation_time)
        {
		    _q_goal_forward[0] = 8.91*DEG2RAD; //q0
		    _q_goal_forward[1] = -147.88*DEG2RAD; //q1
		    _q_goal_forward[2] = 137.98*DEG2RAD; //q2
		    _height_goal_linear = 1.39 - 0.726; //z 
        }       
        else if((time >= _init_time + _operation_time) && (time < _init_time + _operation_time + 5.0))
        {
		    _q_goal_forward[0] = 7.27*DEG2RAD; //q0
		    _q_goal_forward[1] = -148.79*DEG2RAD; //q1
		    _q_goal_forward[2] = 140.43*DEG2RAD; //q2
		    _height_goal_linear = 1.39 - 0.726; //z 
        }
        else if(time >= _init_time + _operation_time + 5.0)
        {
            if(_CurrentTask == Test_Task)
            {
                _CurrentState = Item_Move;
                _init_time = time;              
            }
            _PreviousState = Table_F7;
        }   
        Scara_forward.write_cmd_from_FSM(_operation_time, _q_goal_forward, _x_goal_forward, _bool_ee_control);
        Linear_forward.write_cmd_from_FSM(_operation_time, _height_goal_linear);
        tray_forward.write_cmd_from_FSM(5.0,0.0);
    }

    // Table F8
    else if(_CurrentState == Table_F8)
    {
        _operation_time = 5.0; 

        if(time >= _init_time && time < _init_time + _operation_time)
        {
		    _q_goal_forward[0] = 8.91*DEG2RAD; //q0
		    _q_goal_forward[1] = -147.88*DEG2RAD; //q1
		    _q_goal_forward[2] = 137.98*DEG2RAD; //q2
		    _height_goal_linear = 1.39 - 0.726; //z 
        }       
        else if((time >= _init_time + _operation_time) && (time < _init_time + _operation_time + 5.0))
        {
		    _q_goal_forward[0] = 7.27*DEG2RAD; //q0
		    _q_goal_forward[1] = -148.79*DEG2RAD; //q1
		    _q_goal_forward[2] = 140.43*DEG2RAD; //q2
		    _height_goal_linear = 1.39 - 0.726; //z 
        }
        else if(time >= _init_time + _operation_time + 5.0)
        {
            if(_CurrentTask == Test_Task)
            {
                _CurrentState = Item_Move;
                _init_time = time;              
            }
            _PreviousState = Table_F8;
        }   
        Scara_forward.write_cmd_from_FSM(_operation_time, _q_goal_forward, _x_goal_forward, _bool_ee_control);
        Linear_forward.write_cmd_from_FSM(_operation_time, _height_goal_linear);
        tray_forward.write_cmd_from_FSM(5.0,0.0);
    }

    // Item Move
    else if(_CurrentState == Item_Move)
    {
        int tray_vel= 0.0;

        _operation_time = 6.0; 
        if(time < _init_time + _operation_time)
        {
            if(_PreviousState == (Table_F1 || Table_F2 || Table_F3 || Table_F4 || Table_F5))
            {
                tray_vel = 100;
            }
            else if(_PreviousState == (Table_F6 || Table_F7 || Table_F8 || Drop_Point))
            {
                tray_vel = -100;
            }
        }
        else if(time >= _init_time + _operation_time)
        {
            if(_PreviousState == Table_F1)
            {
                _CurrentState = Table_F1_Lane;
                _init_time = time;              
            }
            else if(_PreviousState == Table_F2)
            {
                _CurrentState = Table_F2_Lane;
                _init_time = time;              
            }
            else if(_PreviousState == Table_F3)
            {
                _CurrentState = Table_F3_Lane;
                _init_time = time;              
            }
            else if(_PreviousState == Table_F4)
            {
                _CurrentState = Table_F4_Lane;
                _init_time = time;              
            }
            else if(_PreviousState == Table_F5)
            {
                _CurrentState = Table_F5_Lane;
                _init_time = time;              
            }
            else if(_PreviousState == Table_F6)
            {
                _CurrentState = Table_F6_Lane;
                _init_time = time;              
            }
            else if(_PreviousState == Table_F7)
            {
                _CurrentState = Table_F7_Lane;
                _init_time = time;              
            }
            else if(_PreviousState == Table_F8)
            {
                _CurrentState = Table_F8_Lane;
                _init_time = time;              
            }
            else if(_PreviousState == Drop_Point)
            {
                _CurrentState = Drop_Lane;
                _init_time = time;
            }           
            _PreviousState = Item_Move;
        }       

        Scara_forward.write_cmd_from_FSM(_operation_time, _q_goal_forward, _x_goal_forward, _bool_ee_control);
        Linear_forward.write_cmd_from_FSM(_operation_time, _height_goal_linear);
        tray_forward.write_cmd_from_FSM(_operation_time, tray_vel);
    }

    // Drop Lane
    else if(_CurrentState == Drop_Lane)
    {
        _operation_time = 10.0;
        if(time < _init_time + _operation_time)
        {
		    _q_goal_forward[0] = -73.0*DEG2RAD; //q0
		    _q_goal_forward[1] = -20.0*DEG2RAD; //q1
		    _q_goal_forward[2] = -87.0*DEG2RAD; //q2
		    _height_goal_linear = 1.39 - 1.262; //z 
        }
        else if(time >= _init_time + _operation_time)
        {
            _CurrentState = Drop_Point;
            _init_time = time;          
            _PreviousState = Drop_Lane;
        }       

        Scara_forward.write_cmd_from_FSM(_operation_time, _q_goal_forward, _x_goal_forward, _bool_ee_control);
        Linear_forward.write_cmd_from_FSM(_operation_time, _height_goal_linear);
        tray_forward.write_cmd_from_FSM(5.0,0.0);
    }

    // Drop Point
    else if(_CurrentState == Drop_Point)
    {
        _operation_time = 10.0;
        if(time < _init_time + _operation_time)
        {
		    _q_goal_forward[0] = -73.0*DEG2RAD; //q0
		    _q_goal_forward[1] = -20.0*DEG2RAD; //q1
		    _q_goal_forward[2] = -87.0*DEG2RAD; //q2
		    _height_goal_linear = 1.39 - 1.262; //z 
        }
        else if(time >= _init_time + _operation_time)
        {
            _CurrentState = Item_Move;
            _init_time = time;          
            _PreviousState = Drop_Point;
        }       

        Scara_forward.write_cmd_from_FSM(_operation_time, _q_goal_forward, _x_goal_forward, _bool_ee_control);
        Linear_forward.write_cmd_from_FSM(_operation_time, _height_goal_linear);
        tray_forward.write_cmd_from_FSM(5.0,0.0);
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

CScara::CScara()
{
    //initialize
    for(int i=0; i<3; i++)
    {
        _q[i] = 0.0;
        _q_goal[i] = 0.0;
        _x_ee_local[i] = 0.0;
        _x_ee_goal_local[i] = 0.0;
        
    }
    _bool_ee_control = false;
    _time_motion = 5.0;
    _link_length[0] = 0.35;
    _link_length[1] = 0.35;
    _link_length[2] = 0.2;

    _rev = true;

    x = 0.0;
    y = 0.0;
    z = 0.0;
    cos_q2 = 0.0;
    sin_q2_1 = 0.0;
    sin_q2_2 = 0.0;
    q1_1 = 0.0;
    q1_2 = 0.0;
    q2_1 = 0.0;
    q2_2 = 0.0;
    q3_1 = 0.0;
    q3_2 = 0.0;
    k1_1 = 0.0;
    k1_2 = 0.0;
    k2_1 = 0.0;
    k2_2 = 0.0;
    gamma_1 = 0.0;
    gamma_2 = 0.0;
}

CScara::~CScara()
{
}

void CScara::update_forward_kinematics()
{
    _x_ee_local[0] = _link_length[0] * cos(_q[0]) + _link_length[1] * cos(_q[0] + _q[1]) + _link_length[2] * cos(_q[0] + _q[1] + _q[2]); // m
	_x_ee_local[1] = _link_length[0] * sin(_q[0]) + _link_length[1] * sin(_q[0] + _q[1]) + _link_length[2] * sin(_q[0] + _q[1] + _q[2]); // m
	_x_ee_local[2] = _q[0] + _q[1] + _q[2]; // rad
}

void CScara::calc_iversekinematics(double IK_x, double IK_y, double IK_a) 
{
    x = 0.0;
    y = 0.0;
    z = 0.0;
    cos_q2 = 0.0;
    sin_q2_1 = 0.0;
    sin_q2_2 = 0.0;
    q1_1 = 0.0;
    q1_2 = 0.0;
    q2_1 = 0.0;
    q2_2 = 0.0;
    q3_1 = 0.0;
    q3_2 = 0.0;
    k1_1 = 0.0;
    k1_2 = 0.0;
    k2_1 = 0.0;
    k2_2 = 0.0;
    gamma_1 = 0.0;
    gamma_2 = 0.0;

	x = IK_x - _link_length[2] * cos(IK_a);
	y = IK_y - _link_length[2] * sin(IK_a);
    
	cos_q2 = (pow(x, 2) + pow(y, 2) - pow(_link_length[0], 2) - pow(_link_length[1], 2)) / (2.0 * _link_length[0] * _link_length[1]);

	if (abs(cos_q2) > 1) {
		std::cout << "Out of Workspace." << endl;
		_x_ee_goal_local[0] = 0.0;// {0.0, 0.0, 0.0 };
		_x_ee_goal_local[1] = 0.0;
		_x_ee_goal_local[2] = 0.0;
	}
	else {

		sin_q2_1 = sqrt(1 - pow(cos_q2, 2));
		sin_q2_2 = -sqrt(1 - pow(cos_q2, 2));

		q2_1 = atan2(sin_q2_1, cos_q2);
		q2_2 = atan2(sin_q2_2, cos_q2);
		
		k1_1 = _link_length[1] * cos(q2_1) + _link_length[0];
		k1_2 = _link_length[1] * cos(q2_2) + _link_length[0];

		k2_1 = _link_length[1] * sin(q2_1);
		k2_2 = _link_length[1] * sin(q2_2);
		
		gamma_1 = atan2(k2_1, k1_1);
		gamma_2 = atan2(k2_2, k1_2);
		
		z = atan2(y, x);
		q1_1 = z - gamma_1;
		q1_2 = z - gamma_2;
		q3_1 = IK_a - q1_1 - q2_1;
		q3_2 = IK_a - q1_2 - q2_2;

        if (_rev == true)// 1 이냐 2냐 고를수 있음
		{
			_x_ee_goal_local[0] = range(q1_2);
            _x_ee_goal_local[1] = range(q2_2);
            _x_ee_goal_local[2] = range(q3_2);
		}
		else
		{
			_x_ee_goal_local[0] = range(q1_1);
            _x_ee_goal_local[1] = range(q2_1);
            _x_ee_goal_local[2] = range(q3_1);
		}
	}

}

double CScara::range(double angle) 
{
	while (angle > PI || angle <= -PI) 
	{
		if (angle > PI) 
		{
			angle = angle - 2 * PI;
		}
		else 
		{
			angle = angle + 2 * PI;
		}
	}
	return angle;
}


void CScara::read_state(double jointangle[]) //read from ROS
{
    _q[0] = jointangle[0]*DEG2RAD;
    _q[1] = jointangle[1]*DEG2RAD;
    _q[2] = jointangle[2]*DEG2RAD;

    update_forward_kinematics();
}

void CScara::write_cmd_from_FSM(double motion_time, double jointangle_cmd[], double endeffector_cmd[], bool bool_endeffector_ctrl) //read from FSM
{
    _time_motion = motion_time;

    _q_goal[0] = jointangle_cmd[0]*RAD2DEG;
    _q_goal[1] = jointangle_cmd[1]*RAD2DEG;
    _q_goal[2] = jointangle_cmd[2]*RAD2DEG;

    _x_ee_goal_local[0] = endeffector_cmd[0];
    _x_ee_goal_local[1] = endeffector_cmd[1];
    _x_ee_goal_local[2] = endeffector_cmd[2];

    _bool_ee_control = bool_endeffector_ctrl;
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

CLinear::CLinear()
{
    _height = 1.39;
    _height_goal = 1.39;
    _time_motion = 10.0;
}

CLinear::~CLinear()
{
    
}

void CLinear::read_state(double height) //read robot state
{
    _height = 1.39 - height; //m, initial height is 1.39 = 0 in encoder
}
    
void CLinear::write_cmd_from_FSM(double motion_time, double height_cmd) //read command
{
    _height_goal = height_cmd - 1.39; //height based on the motor coordinate (0~-1.39)
    _time_motion = motion_time;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

CTray::CTray()
{
    _velocity_cmd = 0.0;
    _time_motion = 5.0;
}

CTray::~CTray()
{
}

void CTray::write_cmd_from_FSM(double motion_time, double velocity_cmd)
{
    _velocity_cmd = velocity_cmd;
    _time_motion = motion_time;
}
