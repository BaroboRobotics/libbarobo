/* Filename: simple.cpp
 * Rotate the faceplates by 90 degrees */
#include <mobot.h>
#include <stdlib.h>

void playBack(CMobot* mobot,
    double* time,
    double* angle1,
    double* angle2,
    double* angle3,
    double* angle4,
    int numPoints);

void saveFile(double* time,
    double* angle1,
    double* angle2,
    double* angle3,
    double* angle4,
    int num,
    const char* filename);

void loadFile(
    double **time,
    double **angle1,
    double **angle2,
    double **angle3,
    double **angle4,
    int *num,
    const char *filename
    );

int main()
{
  double *time;
  double *angle1;
  double *angle2;
  double *angle3;
  double *angle4;
  int numPoints;
  int i;
  CMobot mobot;

  /* Connect to the paired MoBot */
  mobot.connect();
  printf("Press any key to begin recording.\n");
  getchar();
  mobot.recordAnglesBegin(
      time,
      angle1,
      angle2,
      angle3,
      angle4,
      0.0, // period
      0);
  printf("Recording started. Press any key to stop. \n");
  getchar();
  mobot.recordAnglesEnd(numPoints);

  playBack( &mobot,
      time, 
      angle1,
      angle2,
      angle3,
      angle4,
      numPoints);

  saveFile(time, 
      angle1,
      angle2,
      angle3,
      angle4,
      numPoints,
      "test.data");

  loadFile(
      &time,
      &angle1,
      &angle2,
      &angle3,
      &angle4,
      &numPoints,
      "test.data");
  for(i = 0; i < numPoints; i++) {
    printf("%lf, %lf, %lf, %lf, %lf\n",
        time[i],
        angle1[i],
        angle2[i],
        angle3[i],
        angle4[i]);
  }

  return 0;
}

void playBack(CMobot* mobot,
    double* time,
    double* angle1,
    double* angle2,
    double* angle3,
    double* angle4,
    int numPoints)
{
  int i;
  mobot->moveTo(
      angle1[0],
      angle2[0],
      angle3[0],
      angle4[0]);
  for(i = 0; i < numPoints; i++) {
    mobot->driveToNB(
        angle1[i],
        angle2[i],
        angle3[i],
        angle4[i]);
  }
}

void saveFile(double* time,
    double* angle1,
    double* angle2,
    double* angle3,
    double* angle4,
    int num,
    const char* filename)
{
  int i;
  FILE *fp;
  fp = fopen(filename, "w");
  if(fp == NULL) {
    fprintf(stderr,"Error opening file %s for writing\n", filename);
    return;
  }
  fprintf(fp, "%d\n", num);
  for(i = 0; i < num; i++) {
    fprintf(fp, "%lf %lf %lf %lf %lf\n", time[i],
        angle1[i],
        angle2[i],
        angle3[i],
        angle4[i]
        );
  }
  fclose(fp);
}

void loadFile(
    double **time,
    double **angle1,
    double **angle2,
    double **angle3,
    double **angle4,
    int *num,
    const char *filename
    )
{
  int i;
  FILE *fp;
  fp = fopen(filename, "r");
  if(fp == NULL) {
    fprintf(stderr,"Error opening file %s for reading\n", filename);
    return;
  }
  fscanf(fp, "%d\n", num);
  *time = (double*)malloc(sizeof(double) * (*num+1));
  *angle1 = (double*)malloc(sizeof(double) * (*num+1));
  *angle2 = (double*)malloc(sizeof(double) * (*num+1));
  *angle3 = (double*)malloc(sizeof(double) * (*num+1));
  *angle4 = (double*)malloc(sizeof(double) * (*num+1));
  for(i = 0; i < *num; i++) {
    fscanf(fp, "%lf %lf %lf %lf %lf\n", &(*time)[i],
        &(*angle1)[i],
        &(*angle2)[i],
        &(*angle3)[i],
        &(*angle4)[i]
        );
  }
  fclose(fp);
}
