package strapDown;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

class test{
	public static void main(String[] args) throws IOException{
		Motorcycle m = new Motorcycle();
		m.pos.x = 138.9285001;
		m.pos.y = 35.36686017;
		m.gyro.z = 89.068268;
		StrapDownNaruoka s = new StrapDownNaruoka(m);

//		double[][] a2 = {{1,2,3,4},
//						{4,5,6,6}};
//		double[][] b2 = {{1,2},
//						{4,5},
//						{7,8}};
//
//		Matrix matrix = new Matrix();
//		double[][] c2 = matrix.mul(a2, b2);
//		for(int i = 0; i < c2.length; i++){
//			for(int j = 0; j < c2[0].length; j++)
//				System.out.print(c2[i][j] +",");
//			System.out.println();
//		}
		KalmanFilter k = new KalmanFilter();
//		k.timeUpdate();
//		double[][] aa;
//		aa = k.p;
//		for(int i=0;i<aa.length;i++){
//			for(int j=0;j<aa[0].length;j++){
//				System.out.print(aa[i][j] + ",");
//			}
//			System.out.println();
//		}
//		System.exit(1);

		IMU imu = new IMU();
		String imu_line;
		File f = new File("C:\\work\\complete\\Log_07-suzuki1_data_2min.csv");
		FileReader fr;
		fr = new FileReader(f);
		BufferedReader br = new BufferedReader(fr);
		imu_line = br.readLine();
		imu_line = br.readLine();

	    Pattern p_imu_active = Pattern.compile("([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*)");
        Matcher ma;
        double tmp = 0;
        ThreeAxis a = new ThreeAxis();
        ThreeAxis b = new ThreeAxis();
//        System.out.println("vx_strap,vy_strap,vz_strap,vx_sum,vy_sum,vz_sum,roll_strap,pitch_strap,yaw_strap,roll_sum,pitch_sum,yaw_sum,");
        while(imu_line != null){
			ma = p_imu_active.matcher(imu_line);
			if(ma.matches()){
//				kitani
//				imu.acc.x = Double.valueOf(ma.group(59));//bg BaX
//				imu.acc.y = Double.valueOf(ma.group(60));//bh BaY
//				imu.acc.z = Double.valueOf(ma.group(61));//bi BaZ
//				imu.gyro.x = Double.valueOf(ma.group(62));//bj BgX
//				imu.gyro.y = Double.valueOf(ma.group(63));//bk BgY
//				imu.gyro.z = Double.valueOf(ma.group(64));//bl BgZ
//				jigikai
				imu.acc.x = Double.valueOf(ma.group(16));//p x方向加速度[m/s^2]
				imu.acc.y = Double.valueOf(ma.group(17));//q y方向加速度[m/s^2]
				imu.acc.z = Double.valueOf(ma.group(18));//r z方向加速度[m/s^2]
				imu.gyro_v.x = Double.valueOf(ma.group(5)) * Math.PI /180;//e ロールレイト[deg/s]→[rad/s]
				imu.gyro_v.y = Double.valueOf(ma.group(6)) * Math.PI /180;//f ピッチレイト[deg/s]→[rad/s]
				imu.gyro_v.z = Double.valueOf(ma.group(7)) * Math.PI /180;//g ヨーレイト[deg/s]→[rad/s]
				if(Double.isNaN(imu.acc.x)){
					imu_line = br.readLine();
					continue;
				}
//				if(tmp == 1000){
//					break;
//				}
				m = s.main(imu);
				a.x += imu.acc.x * 0.01;
				a.y += imu.acc.y * 0.01;
				a.z += imu.acc.z * 0.01;
				b.x += imu.gyro_v.x * 0.01;
				b.y += imu.gyro_v.y * 0.01;
				b.z += imu.gyro_v.z * 0.01;
//				System.out.print((m.v.x*3.6)+","+(m.v.y*3.6)+","+(m.v.z*3.6)+",");
				System.out.print((m.v.x*3.6)+","+(a.x*3.6)+","+(m.v.y*3.6)+","+(a.y*3.6)+","+(m.v.z*3.6)+","+(a.z*3.6)+",");
//				System.out.print((m.gyro.x/Math.PI*180)+","+(m.gyro.y/Math.PI*180)+","+(m.gyro.z/Math.PI*180)+"\n");
				System.out.print((m.gyro.x/Math.PI*180)+","+(b.x/Math.PI*180)+","+(m.gyro.x/Math.PI*180)+","+(b.y/Math.PI*180)+","+(m.gyro.x/Math.PI*180)+","+(b.z/Math.PI*180)+",");
				System.out.print((m.pos.x/Math.PI*180)+","+(m.pos.y/Math.PI*180)+"\n");

//				s = k.main(s);
			}
			imu_line = br.readLine();
			tmp++;
		}
	}
}