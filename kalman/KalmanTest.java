 /*******************************************************************************
  *
  *  JKalman - KALMAN FILTER (Java) TestBench
  *
  *  Copyright (C) 2007 Petr Chmelar
  *
  *  By downloading, copying, installing or using the software you agree to
  *  the license in licenseIntel.txt or in licenseGNU.txt
  *
  **************************************************************************** */

package kalman;

import jama.Matrix;

import java.util.Random;

import jkalman.JKalman;



/**
 * JKalman TestBench
 */
public class KalmanTest {
    /**
     * Constructor
     */
    public KalmanTest() {
    }

    /**
     * Main method
     * @param args
     * @throws Exception
     */
    public static void main(String[] args) throws Exception {

		JKalman kalman = new JKalman(3, 1);

		// init
		Matrix s = new Matrix(3, 1); // state [dx,x]
		Matrix c = new Matrix(3, 1); // corrected state [dx, x]

		Matrix m = new Matrix(1, 1); // measurement [dx]

		// transitions for dx,x
//		double[][] tr = { {1, 0},
//				      {1, 1}};
		double[][] tr = {{1, 0,1},
			      		 {1, 1,0},
			      		 {0, 0,1}};
		kalman.setTransition_matrix(new Matrix(tr));

		// 1s somewhere?
		kalman.setError_cov_post(kalman.getError_cov_post().identity());

		System.out.println("time,roll_rate,roll_sum,prediction_roll_rate,prediction_roll,correction_roll_rate,correction_roll,bias");

		double time_range = 100;
		double[] roll_rate = new double[(int)time_range*20];//dxの観測値の値
		double[] roll = new double[(int)time_range*20];//rollの観測値．真値
		double roll_true = 0;
		//roll_rateをつくる
		//元データ+ホワイトノイズ+オフセット
		double noise_size = 1.0;
		//オフセットは a*i+bの一次
		double offset_a = 0.01;
		double offset_b = 0.1;
//		noise_size = 0;
//		offset_a = 0;
//		offset_b=0;

		Random r = new Random();
		r.setSeed(1);
		for(int i = 0; i < time_range; i ++){
			roll_rate[i] =(45.0 / (time_range*2)) * ((double)i / time_range) + (r.nextDouble() * noise_size - noise_size/2) + (offset_a * i + offset_b);
			roll_true   +=(45.0 / (time_range*2)) * ((double)i / time_range);
			roll[i] = roll_true;
		}
		for(int i = (int)time_range; i < time_range*2; i++){
			roll_rate[i] = 45.0 / (time_range*2) + (r.nextDouble() * noise_size - noise_size/2) + (offset_a * i + offset_b);
			roll_true   +=(45.0 / (time_range*2));
			roll[i] = roll_true;
		}
		for(int i = (int)time_range*2; i < time_range*3; i++){
			roll_rate[i] = (45.0 / (time_range*2)) -  45.0 / (time_range*2) * ((double)(i-time_range*2) / time_range) + (r.nextDouble() * noise_size - noise_size/2) + (offset_a * i + offset_b);
			roll_true   += (45.0 / (time_range*2)) -  45.0 / (time_range*2) * ((double)(i-time_range*2) / time_range);
			roll[i] = roll_true;
		}
		for(int i = (int)time_range*3; i < time_range*17; i++){
			roll_rate[i] = 0  + (r.nextDouble() * noise_size - noise_size/2) + (offset_a * i + offset_b);
			roll_true   += 0;
			roll[i] = roll_true;
		}
		for(int i = (int)time_range*17; i < time_range*18; i++){
			roll_rate[i] =(-45.0 / (time_range*2)) * ((double)(i-time_range*17) / time_range) + (r.nextDouble() * noise_size - noise_size/2) + (offset_a * i + offset_b);
			roll_true   +=(-45.0 / (time_range*2)) * ((double)(i-time_range*17) / time_range);
			roll[i] = roll_true;
		}
		for(int i = (int)time_range*18; i < time_range*19; i++){
			roll_rate[i] =-45.0 /(time_range*2) + (r.nextDouble() * noise_size - noise_size/2) + (offset_a * i + offset_b);
			roll_true   +=-45.0 /(time_range*2);
			roll[i] = roll_true;
		}
		for(int i = (int)time_range*19; i < time_range*20; i++){
			roll_rate[i] =-45.0 / (time_range*2) +  (45.0 / (time_range*2)) * ((double)(i-time_range*19) / time_range) + (r.nextDouble() * noise_size - noise_size/2) + (offset_a * i + offset_b);
			roll_true   +=-45.0 / (time_range*2) +  (45.0 / (time_range*2)) * ((double)(i-time_range*19) / time_range);
			roll[i] = roll_true;
		}

		// For debug only
		double roll_sum = 0;//roll_rateの積算によるroll
//		double[][] a = {{0,0},{0,0}};
//		Matrix process_noise_cov =new Matrix(a);
//		kalman.setProcess_noise_cov(process_noise_cov);
		for (int i = 0; i < time_range*20; ++i) {
		    s = kalman.Predict();

		    m.set(0,0,roll_rate[i]-c.get(2, 0));
//		    m.set(1,0,roll[i]);

		    c = kalman.Correct(m);
		    roll_sum += m.get(0, 0);
//		    System.out.println("" + i + "," +  m.get(0, 0) + "," + m.get(1, 0) + ","
		    System.out.println("" + i + "," +  m.get(0, 0) + "," + roll_sum + ","
				 + s.get(0, 0) + "," + s.get(1, 0) + ","
				 + c.get(0, 0) + "," + c.get(1, 0) + "," + c.get(2, 0));

		}
    }
}
