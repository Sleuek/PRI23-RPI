#include <sys/time.h>
#include <stdint.h>
#include <unistd.h>
#include <math.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <time.h>
#include <unistd.h>
#include "IMU.c"


#define DT 0.02         // [s/loop] loop period. 20ms
#define G_GAIN 0.070     // [deg/s/LSB]
#define G_accelero 0.000244 //[mg/LSB]
#define seuil 0.5 // Seuil [g]
#define seuil_basculement 0.8 // Seuil Basculement [g]
#define hysteresis 0.05
 




void  INThandler(int sig)// Used to do a nice clean exit when Ctrl-C is pressed
{
	signal(sig, SIG_IGN);
	exit(0);
}

int mymillis()
{
	struct timeval tv;
	gettimeofday(&tv, NULL);
	return (tv.tv_sec) * 1000 + (tv.tv_usec)/1000;
}

int timeval_subtract(struct timeval *result, struct timeval *t2, struct timeval *t1)
{
    long int diff = (t2->tv_usec + 1000000 * t2->tv_sec) - (t1->tv_usec + 1000000 * t1->tv_sec);
    result->tv_sec = diff / 1000000;
    result->tv_usec = diff % 1000000;
    return (diff<0);
}

int main(int argc, char *argv[])
{


	float rate_gyr_y = 0.0;   // [deg/s]
	float rate_gyr_x = 0.0;   // [deg/s]
	float rate_gyr_z = 0.0;   // [deg/s]
	
	float value_accelerometer_X, value_accelerometer_Y, value_accelerometer_Z;

	int  accRaw[3];
	int  gyrRaw[3];



	float gyroXangle = 0.0;
	float gyroYangle = 0.0;
	float gyroZangle = 0.0;

	int rafraich_period=100;

	float norme;
	int count; 
	

	int startInt  = mymillis();
	struct  timeval tvBegin, tvEnd,tvDiff;


	signal(SIGINT, INThandler);

	detectIMU();
	enableIMU();

	gettimeofday(&tvBegin, NULL);
 	
	


	while(1)
	{
		startInt = mymillis();


		//read ACC and GYR data
		readACC(accRaw);
		readGYR(gyrRaw);

		//Convert Gyro raw to degrees per second
		rate_gyr_x = (float) gyrRaw[0]  * G_GAIN;
		rate_gyr_y = (float) gyrRaw[1]  * G_GAIN;
		rate_gyr_z = (float) gyrRaw[2]  * G_GAIN;



		//Calculate the angles from the gyro
		gyroXangle = (gyroXangle +rate_gyr_x*DT);
		gyroYangle = (gyroYangle +rate_gyr_y*DT);
		gyroZangle = (gyroZangle +rate_gyr_z*DT);



		//Get raw values of accelerometer
		
		 value_accelerometer_X=accRaw[0]*G_accelero;
		 value_accelerometer_Y=accRaw[1]*G_accelero;
		 value_accelerometer_Z=accRaw[2]*G_accelero;

		// Norm 

		norme = sqrt(value_accelerometer_X*value_accelerometer_X+value_accelerometer_Y*value_accelerometer_Y+((value_accelerometer_Z-1)*(value_accelerometer_Z-1)));
		
		// Test 
		
		if(value_accelerometer_X > seuil || value_accelerometer_X < -seuil)
		{
			printf("value_accelerometer_X: %f \n",value_accelerometer_X);
			printf("norme : %f \n", norme);
			usleep(100);
		}

		if(value_accelerometer_Y > seuil || value_accelerometer_Y < -seuil)
		{
			printf("value_accelerometer_Y: %f \n",value_accelerometer_Y);
			printf("norme : %f \n", norme);
			usleep(100);
		}
		
		// Basculement 
		
		if (value_accelerometer_Z < seuil_basculement-hysteresis && value_accelerometer_Z > -seuil_basculement+hysteresis)
		{
			count ++;
			
			if (count > rafraich_period)
			{
				count =0;
				printf("Basculement detected\n");
			}
			
		}

	
		else if (value_accelerometer_Z > seuil_basculement+hysteresis || value_accelerometer_Z < -seuil_basculement-hysteresis)
		{
			
			printf("Stable selon z\n");
			printf("%f\n",value_accelerometer_Z);
			count =0;
		}

		//printf("value_accelerometer_Z: %f\n", value_accelerometer_Z);

		//printf ("gyroXangle: %f ° \ngyroYangle: %f ° \ngyroZangle: %f ° \nvalue_accelerometer_X: %f g\nvalue_accelerometer_Y: %f g\nvalue_accelerometer_Z: %f g\n",gyroXangle,gyroYangle,gyroZangle,value_accelerometer_X,value_accelerometer_Y,value_accelerometer_Z);
		//sleep(1);
		//usleep(20);

		//Each loop should be at least 20ms.
		while(mymillis() - startInt < (DT*1000)){
				usleep(100);
		}

		//printf("Loop Time %d\n", mymillis()- startInt);
    }
}

