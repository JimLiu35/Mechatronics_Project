#include "Getcoordinates.h"
using namespace std;
void Getcoordinates(char * message,float*res){
  char* test = message;


  // 46 .
  // 44 ,
  // 49 1
  float first_x = int(test[1] - 48) * 10 + int(test[2] - 48) + int(test[3] - 48) * 0.1 + int(test[4] - 48)* 0.01;
  
  float first_y = int(test[6] - 48) * 10 + int(test[7] - 48) + int(test[8] - 48) * 0.1 + int(test[9] - 48)* 0.01;

  float second_x = int(test[11] - 48) * 10 + int(test[12] - 48) + int(test[13] - 48) * 0.1 + int(test[14] - 48)* 0.01;
  
  float second_y = int(test[16] - 48) * 10 + int(test[17] - 48) + int(test[18] - 48) * 0.1 + int(test[19] - 48)* 0.01;

  float third_x = int(test[21] - 48) * 10 + int(test[22] - 48) + int(test[23] - 48) * 0.1 + int(test[24] - 48)* 0.01;
  
  float third_y = int(test[26] - 48) * 10 + int(test[27] - 48) + int(test[28] - 48) * 0.1 + int(test[29] - 48)* 0.01;
  res[0] = first_x;
  res[1] = first_y;
  res[2] = second_x;
  res[3] = second_y;
  res[4] = third_x;
  res[5] = third_y;

}

int start_stop_message(char * message){
  // get first char
  // ! 33 start return 1
  // ? 63 stop return 2
  // b or g message return 3
  char* test = message;
  if (int(test[0]) == 33){
    // start
    return 1;
  }
  else if (int(test[0]) == 63){
    // stop
    return 2;
  }
  else{
    return 3;
  }
}
