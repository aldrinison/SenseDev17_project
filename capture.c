#include <stdio.h>

int main(){
  FILE* proc1 = popen("java net.tinyos.tools.PrintfClient -comm serial@/dev/ttyUSB0:telosb", "r");
  char buff[255];

  FILE* data;
  // initial printf outputs
  
  while (1){
    fscanf(proc1, "%s", buff);
    printf("%s\n", buff);
    
    //filtering
    if ((buff[0] == 'X') && (buff[1] == 'Y') && (buff[2] == 'Z')) {
      data = fopen("/vagrant/datadump.txt", "w+");
      //printf("File is open\n");
      fprintf(data, "%s", buff);      
      fclose(data);
      //printf("File closed\n");
    }
  }

  pclose(proc1);
  
  return 0;
}
