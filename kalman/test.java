package kalman;

class test{
	//HPF(gyro)
	static double Q_angle  =  0.001;
	static double Q_gyro   =  0.003;
	static double R_angle  =  0.03;
	static double x_angle;
	static double x_bias = 0;
	static double P_00 = 0, P_01 = 0, P_10 = 0, P_11 = 0;
	static double  y, S;
	static double K_0, K_1;
	static double dt=0.005;
	static double kal_deg;

    public static void main(String[] args){
    	double angle = 0;
    	angle = kalmanCalculate(angle,3);
    	System.out.println(angle);
    }


	static double kalmanCalculate(double newAngle, double newRate){

	        x_angle += dt * (newRate - x_bias);

	        P_00 +=  dt * (dt * P_11 - P_01 - P_10 + Q_angle);
	        P_01 -=  dt * P_11;
	        P_10 -=  dt * P_11;
	        P_11 +=  Q_gyro * dt;

	        y = newAngle - x_angle;
	        S = P_00 + R_angle;
	        K_0 = P_00 / S;
	        K_1 = P_10 / S;

	        x_angle +=  K_0 * y;
	        x_bias  +=  K_1 * y;
	        P_00 -= K_0 * P_00;
	        P_01 -= K_0 * P_01;
	        P_10 -= K_1 * P_00;
	        P_11 -= K_1 * P_01;

	    return x_angle;
	}

}