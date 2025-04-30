/* 
 * Originally by Philip Koopman (koopman@cmu.edu)
 * and Milda Zizyte (milda@cmu.edu)
 *
 * STUDENT NAME:
 * ANDREW ID:
 * LAST UPDATE:
 *
 * This file is an algorithm to solve the ece642rtle maze
 * using the left-hand rule. The code is intentionaly left obfuscated.
 *
 */

#include "student.h"

// Ignore this line until project 5
turtleMove studentTurtleStep(bool bumped) {return MOVE;}

// OK TO MODIFY BELOW THIS LINE

#define TIMEOUT 100    // bigger number slows down simulation so you can see what's happening
float timer;
int turtle_state = 1;
float fx1, fy1, fx2, fy2;
bool end_flag, bumped_flag;
enum TurtleState {
	COLLISION = 0,
	JUST_MOVED = 1,
	MOVE_FORWARD = 2
};
enum Orientation {
	LEFT = 0,
	DOWN = 1,
	RIGHT = 2,
	UP = 3
};
		 
// this procedure takes the current turtle position and orientation and returns
// true=submit changes, false=do not submit changes
// Ground rule -- you are only allowed to call the helper functions "bumped(..)" and "atend(..)",
// and NO other turtle methods or maze methods (no peeking at the maze!)
bool studentMoveTurtle(QPointF& pos_, int& new_orient) {   
	ROS_INFO("Turtle update Called  timer=%f", timer);

    if(timer == 0) { 

		fx1 = pos_.x(); fy1 = pos_.y();
    	fx2 = pos_.x(); fy2 = pos_.y();
		
		switch(new_orient){
			case LEFT:
				fy2+=1;
				break;

			case DOWN:
				fx2+=1;
				break;
				
			case RIGHT:
				fx2+=1;
				fy2+=1;
				fx1+=1;
				break;

			case UP:
				fx2+=1;
				fy2+=1;
				fy1+=1;
				break;

			default:
				break;
		}

		bumped_flag = bumped(fx1,fy1,fx2,fy2);
		end_flag = atend(pos_.x(), pos_.y());

		//by switching the new_orient, depending on
		//turtle_state, the turtle may go left or right.
		//Actual config is for right-hand-rule.
		switch(new_orient){
			case LEFT:
				if(turtle_state == MOVE_FORWARD){ 
					new_orient = DOWN;  //UP for left-hand-rule
					turtle_state = JUST_MOVED; 
				}
				else if (bumped_flag){ 
					new_orient = UP;  //DOWN for left-hand-rule
					turtle_state = COLLISION; 
				}
				else turtle_state = MOVE_FORWARD;
				break;

			case DOWN:
				if(turtle_state == MOVE_FORWARD){ 
					new_orient = RIGHT;      //LEFT for left-hand-rule
					turtle_state = JUST_MOVED; 
				}
				else if (bumped_flag) { 
					new_orient = LEFT;  	//RIGHT for left-hand-rule
					turtle_state = COLLISION; 
				}
				else turtle_state = MOVE_FORWARD;
				break;

			case RIGHT:
				if(turtle_state == MOVE_FORWARD){ 
					new_orient = UP;  		//DOWN for left-hand-rule
					turtle_state = JUST_MOVED; 
				}
				else if (bumped_flag){ 
					new_orient = DOWN;  	//UP for left-hand-rule
					turtle_state = COLLISION; 
				}
				else turtle_state = MOVE_FORWARD;
				break;

			case UP:
				if(turtle_state == MOVE_FORWARD){ 
					new_orient = LEFT;  	//RIGHT for left-hand-rule
					turtle_state = JUST_MOVED; 
				}
				else if (bumped_flag) { 
					new_orient = RIGHT; 	//LEFT for left-hand-rule
					turtle_state = COLLISION; 
				}
				else turtle_state = MOVE_FORWARD;
				break;
			
			default:
				break;
		}

		ROS_INFO("Orientation=%f  STATE=%f", new_orient, turtle_state);
		if(turtle_state == MOVE_FORWARD && end_flag == false) {
			switch(new_orient){
				case DOWN:
					pos_.setY(pos_.y() - 1); 
					break;
				case RIGHT:
					pos_.setX(pos_.x() + 1);
					break;
				case UP:
					pos_.setY(pos_.y() + 1); 
					break;
				case LEFT:
					pos_.setX(pos_.x() - 1);
					break;
				default:
					break;
			}
    	}
	}

    if (end_flag) return false;
    if (timer==0) timer  = TIMEOUT; else timer -= 1;
    if (timer==TIMEOUT) return true;
	return false;
}
