/*
 * Copyright 1996-2020 Cyberbotics Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * Description:   Starts with a predefined behaviors and then
 *                read the user keyboard inputs to actuate the
 *                robot
 */

#include <webots/keyboard.h>
#include <webots/robot.h>

#include <arm.h>
#include <base.h>
#include <gripper.h>

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define TIME_STEP 32

static void step() {
  if (wb_robot_step(TIME_STEP) == -1) {
    wb_robot_cleanup();
    exit(EXIT_SUCCESS);
  }
}

static void passive_wait(double sec) {
  double start_time = wb_robot_get_time();
  do {
    step();
  } while (start_time + sec > wb_robot_get_time());
}

// static void grap_front(){
  // passive_wait(2.0);
  // gripper_release();
  // arm_set_height(ARM_FRONT_FLOOR);
  // passive_wait(5.0);
  // gripper_grip();
  // passive_wait(1.0);
  // arm_set_height(ARM_BACK_PLATE_HIGH);
  // passive_wait(3.0);
  // gripper_release();
  // passive_wait(1.0);
  // arm_reset();
// }

// static void grap_right(){
  // passive_wait(2.0);
  // gripper_release();
  // arm_set_orientation(ARM_RIGHT);
  // arm_set_height(ARM_FRONT_FLOOR);
  // passive_wait(5.0);
  // gripper_grip();
  // passive_wait(3.0);
  // arm_set_orientation(ARM_FRONT);
  // arm_set_height(ARM_BACK_PLATE_HIGH);
  // passive_wait(3.0);
  // gripper_release();
  // passive_wait(1.0);
  // arm_reset();
// }

// static void grap_left(){
  // passive_wait(2.0);
  // gripper_release();
  // arm_set_orientation(ARM_LEFT);
  // arm_set_height(ARM_FRONT_FLOOR);
  // passive_wait(5.0);
  // gripper_grip();
  // passive_wait(3.0);
  // arm_set_orientation(ARM_FRONT);
  // arm_set_height(ARM_BACK_PLATE_HIGH);
  // passive_wait(3.0);
  // gripper_release();
  // passive_wait(1.0);
  // arm_reset();
// }
// static void move_Kuka(char d , float t){
  // if(d == 'f'){
    // base_forwards();
  // }
  // else if(d == 'b'){
    // base_backwards();
  // }
  // else if(d == 'r'){
    // base_turn_right();
  // }
  // else {
    // base_turn_left();
  // }
   // passive_wait(t);
// }
// static void directed_move(double x, double y, double z){
  // gripper_release();
  // arm_ik(x,y,z);
  // passive_wait(2.0);
  // gripper_release();
  // arm_set_height(ARM_FRONT_FLOOR);
  // passive_wait(5.0);
  // gripper_grip();
  // passive_wait(1.0);
  // arm_reset();
  // passive_wait(3.0);
  // arm_set_height(ARM_BACK_PLATE_HIGH);
  // passive_wait(3.0);
  // gripper_release();
  // passive_wait(1.0);
  // arm_reset();
// }
// static void automatic_behavior() {
  // grap_front();
  // grap_right();
  // grap_left();
  // move_Kuka('f',2.0f);
  // move_Kuka('b',3.0f);
  // directed_move(0.16f,0.0298225,1.25);
// }

static void display_helper_message() {
  printf("Control commands:\n");
  printf(" Arrows:       Move the robot\n");
  printf(" Page Up/Down: Rotate the robot\n");
  printf(" +/-:          (Un)grip\n");
  printf(" Shift + arrows:   Handle the arm\n");
  printf(" Space: Reset\n");
}
static void manual_drive(){
  int pc = 0;
  wb_keyboard_enable(TIME_STEP);
 
  while (true) {
    step();

    int c = wb_keyboard_get_key();
    if ((c >= 0) && c != pc) {
      printf("%c", c);
      switch (c) {
        case WB_KEYBOARD_UP:
          printf("Go forwards\n");
          base_forwards();
          break;
        case WB_KEYBOARD_DOWN:
          printf("Go backwards\n");
          base_backwards();
          break;
        case WB_KEYBOARD_LEFT:
          printf("Strafe left\n");
          base_strafe_left();
          break;
        case WB_KEYBOARD_RIGHT:
          printf("Strafe right\n");
          base_strafe_right();
          break;
        case WB_KEYBOARD_PAGEUP:
          printf("Turn left\n");
          base_turn_left();
          break;
        case WB_KEYBOARD_PAGEDOWN:
          printf("Turn right\n");
          base_turn_right();
          break;
        case WB_KEYBOARD_END:
        case ' ':
          printf("Reset\n");
          base_reset();
          arm_reset();
          break;
        case '+':
        case 388:
        case 65585:
          printf("Grip\n");
          gripper_grip();
          break;
        case '-':
        case 390:
          printf("Ungrip\n");
          gripper_release();
          break;
        case 332:
        case WB_KEYBOARD_UP | WB_KEYBOARD_SHIFT:
          printf("Increase arm height\n");
          arm_increase_height();
          break;
        case 326:
        case WB_KEYBOARD_DOWN | WB_KEYBOARD_SHIFT:
          printf("Decrease arm height\n");
          arm_decrease_height();
          break;
        case 330:
        case WB_KEYBOARD_RIGHT | WB_KEYBOARD_SHIFT:
          printf("Increase arm orientation\n");
          arm_increase_orientation();
          break;
        case 328:
        case WB_KEYBOARD_LEFT | WB_KEYBOARD_SHIFT:
          printf("Decrease arm orientation\n");
          arm_decrease_orientation();
          break;
        default:
          fprintf(stderr, "Wrong keyboard input\n");
          break;
      }
    }
    pc = c;
  }

}

int main(int argc, char **argv) {
  wb_robot_init();

  base_init();
  arm_init();
  gripper_init();
  passive_wait(2.0);
  // automatic_behavior();
  display_helper_message();
  manual_drive();
  wb_robot_cleanup();

  return 0;
}
