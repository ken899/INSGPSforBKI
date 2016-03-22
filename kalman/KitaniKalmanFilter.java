package kalman;

import jama.Matrix;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

import jkalman.JKalman;
import strapDown.Quaternion;
import strapDown.ThreeAxis;

//卒論に使用
//GPSのドップラーによる測定値を用いてINSの結果を補正
//新木谷研用
//入力観測値：方位角，速度
//状態：速度，加速度，姿勢角，角速度

public class kitaniKalmanFilter {
	static double dt = 0.01;//時間の間隔

	public KitaniKalmanFilter() {
	}

	public static void main(String[] args) throws IOException {
		kalman_imu();
	}
	public static void kalman_imu() {
		File in  = new File("E:\\bike\\complete\\Log_07-suzuki1_data.csv");
		File out = new File("E:\\bike\\complete\\Log_07-suzuki1_imu_kalman.csv");
		String cur = "";

		try {
			//JKalman(状態行列の大きさ，観測行列の大きさ);
			//どちらの行列も正方行列
			JKalman kalman = new JKalman(8, 8);

			// init
			double vx = 0;
			double vy = 0;

			Matrix s = new Matrix(8, 1); // state [vx,vy,ax,ay,roll,yaw,gyro_b.x,gyro_b.z]状態変数
			Matrix c = new Matrix(8, 1); // state [vx,vy,ax,ay,roll,yaw,gyro_b.x,gyro_b.z]更新値の箱

			Matrix m = new Matrix(8, 1); // measurement [vx,vy,ax,ay,roll,yaw,gyro_b.x,gyro_b.z]観測値のフィルタ
			// transitions for vx,vy,ax,ay,roll,yaw,gyro_n.x,gyro_n.z 状態遷移行列
			double[][] tr = { {1, 0, dt, 0,0,0,0,0},
							  {0, 1, 0, dt,0,0,0,0},
							  {0, 0, 1, 0,0,0,0,0},
							  {0, 0, 0, 1,0,0,0,0},
							  {0,0,0,0,1, 0, dt, 0},
							  {0,0,0,0,0, 1, 0, dt},
							  {0,0,0,0,0, 0, 1, 0},
							  {0,0,0,0,0, 0, 0, 1}};
			kalman.setTransition_matrix(new Matrix(tr));

			//観測雑音行列．vx,vy,ax,ay,roll,yaw,roll_rate,yaw_rate.数字が大きい値程重みが小さい
			double[][] noisem = {{Math.pow(2, 2),0,0,0,0,0,0,0},
								{0,Math.pow(2, 2),0,0,0,0,0,0},
								{0,0,Math.pow(0.2, 2),0,0,0,0,0},
								{0,0,0,Math.pow(0.2, 2),0,0,0,0},
								{0,0,0,0,Math.pow(2, 2),0,0,0},
								{0,0,0,0,0,Math.pow(0.2, 2),0,0},
								{0,0,0,0,0,0,Math.pow(2, 2),0},
								{0,0,0,0,0,0,0,Math.pow(2, 2)}};
			kalman.setMeasurement_noise_cov(new Matrix(noisem));
			double[][] noisep = {{Math.pow(0.1, 4),0,0,0,0,0,0,0},
								{0,Math.pow(0.1, 4),0,0,0,0,0,0},
								{0,0,Math.pow(0.1, 4),0,0,0,0,0},
								{0,0,0,Math.pow(0.1, 4),0,0,0,0},
								{0,0,0,0,Math.pow(0.1, 4),0,0,0},
								{0,0,0,0,0,Math.pow(0.1, 4),0,0},
								{0,0,0,0,0,0,Math.pow(0.1,4),0},
								{0,0,0,0,0,0,0,Math.pow(0.1, 4)}};
			kalman.setProcess_noise_cov(new Matrix(noisep));

			kalman.setError_cov_post(kalman.getError_cov_post().identity());

			FileReader fr = new FileReader(in);
			BufferedReader br = new BufferedReader(fr);
			FileWriter fw = new FileWriter(out);
			Pattern p_cmp = Pattern.compile("([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*)");
			//83:lat 85:lon 87:gps_speed 88:gps_dir
			//59:bax 60:bay 62:bgx 63:bgy 64:bgz
			Pattern p_double = Pattern.compile("\\s*[+-]?(\\d+\\.\\d+|\\d+)");
			double yaw=0,yaw_read=0,yaw_read_pre=0;
            Matcher mat;
            Quaternion b2n = new Quaternion();
            ThreeAxis att = new ThreeAxis();//姿勢角.rateではない
            ThreeAxis att_no_kalman = new ThreeAxis();//姿勢角.kalmanフィルタなしの座標変換のみの積算だけ
            ThreeAxis att_b = new ThreeAxis();//B座標系角速度を微分で求めるためのもの
            ThreeAxis v_no_kalman = new ThreeAxis();
            ThreeAxis gyro_b = new ThreeAxis();//rate.b座標系
            ThreeAxis gyro_n = new ThreeAxis();//rate.n座標系
            ThreeAxis acc_b = new ThreeAxis();//加速度.b座標系
            ThreeAxis acc_n = new ThreeAxis();//加速度.n座標系
            ThreeAxis acc_jigikai_b = new ThreeAxis();//自技会データの加速度のb座標系版
            ThreeAxis acc_jigikai_n = new ThreeAxis();//自技会データの加速度のn座標系版
            double vx_k=0,vy_k=0,roll_k=0,yaw_k=0;
            double time_pre = 0;

			cur = br.readLine();//ヘッダ読み飛ばし

			cur = br.readLine();
			mat = p_cmp.matcher(cur);
			while(!mat.matches()){
				cur = br.readLine();
				mat = p_cmp.matcher(cur);
			}

			b2n = getNtoB(att).conjugate();

			fw.write("time,vx,vy,accx_b,accy_b,rollrate_kalman,rollrate_kalman,roll,yaw,rollrate_calc,yawrate_calc,vx_jigikai,vy_jigikai,accx_b_jigikai,accx_n_jigikai,accy_b_jigikai,accy_n_jigikai,roll_jigikai,yaw_jigikai,rollrate_jigikai,yawrate_jigikai,vx_kitani,vy_kitani,accx_kitani,accy_kitani,roll_kitani,yaw_kitani,rollrate_kitani,yawrate_kitani\n");
			String a = "\"\"";
			String b = "";
			while (cur != null) {
				System.out.println(cur);
				s = kalman.Predict();//カルマン予測

				mat = p_cmp.matcher(cur);
				if(mat.matches()){
					if(a.matches(mat.group(3)) || b.matches(mat.group(3))){
						//imuデータが空の時読み飛ばす
						cur = br.readLine();
						continue;
					}
					if(a.matches(mat.group(83)) || b.matches(mat.group(83))){
						//gpsデータが空の時読み飛ばす
						cur = br.readLine();
						continue;
					}

					//経過時間
					dt = Double.parseDouble(mat.group(1)) - time_pre;
					time_pre = Double.parseDouble(mat.group(1));
					tr[0][2] = dt;
					tr[1][3] = dt;
					tr[4][6] = dt;
					tr[5][7] = dt;
					kalman.setTransition_matrix(new Matrix(tr));

					//ヨー角
					yaw_read = Double.parseDouble(mat.group(88)) * Math.PI / 180;
					if(yaw_read - yaw_read_pre > Math.PI * 3 / 2){
						yaw += yaw_read - yaw_read_pre - Math.PI * 2;
					}else if(yaw_read - yaw_read_pre < -Math.PI * 3 /2){
						yaw += yaw_read - yaw_read_pre + Math.PI * 2;
					}else{
						yaw += yaw_read - yaw_read_pre;
					}
					yaw_read_pre = yaw_read;
					att.z = yaw;

					//速度，加速度，角速度
					vx = Double.parseDouble(mat.group(87))/3.6 * Math.sin(att.z);
					vy = Double.parseDouble(mat.group(87))/3.6 * Math.cos(att.z);
					acc_b.x = Double.parseDouble(mat.group(59));
					acc_b.y =-Double.parseDouble(mat.group(60));
					acc_b.z =-Double.parseDouble(mat.group(61));

					acc_n = b2n.rotateFrame(acc_b);

					gyro_b.x = Double.parseDouble(mat.group(62)) * Math.PI / 180;
//					gyro_b.y = Double.parseDouble(mat.group(63)) * Math.PI / 180;
					gyro_b.z =-Double.parseDouble(mat.group(64)) * Math.PI / 180;
					gyro_n = b2n.rotateFrame(b2n,gyro_b);

					//ロール角．要改善
					att.x = Math.atan(gyro_n.z * Math.sqrt(vx*vx+vy*vy) / 9.8);


					//カルマンなしの積算による姿勢角，速度を作る
					b2n = getNtoB(att_no_kalman).conjugate();
					ThreeAxis tmp = new ThreeAxis();
					tmp = b2n.rotateFrame(gyro_b);//rad
					att_no_kalman.x += tmp.x * dt;//rad
					att_no_kalman.z += tmp.z * dt;//rad
					tmp = b2n.rotateFrame(acc_b);
					v_no_kalman.x += tmp.x * dt;
					v_no_kalman.y += tmp.y * dt;

					//カルマン観測，補正
					m.set(0, 0, vx);
					m.set(1, 0, vy);
					m.set(2, 0, acc_n.x);
					m.set(3, 0, acc_n.y);
					m.set(4, 0, att.x);
					m.set(5, 0, att.z);
					m.set(6, 0, gyro_n.x);
					m.set(7, 0, gyro_n.z);

					c = kalman.Correct(m);

					vx = c.get(0,0);
					vy = c.get(1,0);
					acc_n.x = c.get(2, 0);
					acc_n.y = c.get(3, 0);
					att.x = c.get(4, 0);
					att.z = fitPI(c.get(5, 0));
					gyro_n.x = c.get(6, 0);
					gyro_n.z =c.get(7, 0);

					//現在の姿勢角からIMU座標系→GPS座標系変換用の回転クォータニオン算出
					b2n = getNtoB(att).conjugate();

					acc_b = b2n.conjugate().rotateFrame(acc_n);
					gyro_b = b2n.conjugate().rotateFrame(gyro_n);
					ThreeAxis tmp2 = new ThreeAxis();
					tmp2 = att_b;
					att_b = b2n.conjugate().rotateFrame(att);

					//kalman
//					fw.write(mat.group(1) + "," + vx + "," + vy + "," + acc_b.x + "," + acc_n.x + "," + acc_b.y + "," + acc_n.y + "," + (att.x * 180 / Math.PI) + "," + (att.z * 180 / Math.PI) + "," + (gyro_b.x * 180 / Math.PI) + "," + (gyro_b.z * 180 / Math.PI) + ",");
					fw.write(mat.group(1) + "," + vx + "," + vy + "," + acc_b.x + "," + acc_b.y + "," + (gyro_b.x * 180 / Math.PI) + "," + (gyro_b.z * 180 / Math.PI) + "," + (att.x * 180 / Math.PI) + "," + (att.z * 180 / Math.PI) + "," + ( (att_b.x-tmp2.x) * 180 / Math.PI) + "," + ((att_b.z-tmp2.z) * 180 / Math.PI) + ",");
					//vx,vy,accx,accy,roll,yaw,rollrate.yawrate,

					//jigikai
					acc_jigikai_b.x = Double.parseDouble(mat.group(16));
					acc_jigikai_b.y = Double.parseDouble(mat.group(17));
					acc_jigikai_b.z = Double.parseDouble(mat.group(18));
					acc_jigikai_n = b2n.rotateFrame(b2n, acc_jigikai_b);
					fw.write(Double.parseDouble(mat.group(9))/3.6*Math.sin(Double.parseDouble(mat.group(14))/180*Math.PI) + "," + Double.parseDouble(mat.group(9))/3.6*Math.cos(Double.parseDouble(mat.group(14))/180*Math.PI) + ",");
					//vx,vy
					fw.write(acc_jigikai_b.x + "," + acc_jigikai_n.x + ",");
					fw.write(acc_jigikai_b.y + "," + acc_jigikai_n.y + ",");
					//acc_n.x,acc_n.y
					fw.write(mat.group(11) + "," + mat.group(14) + "," );
					//roll,yaw,
					fw.write(mat.group(5) + "," + Double.parseDouble(mat.group(7)) + "," );
					//roll_rate,yaw_rate

					//kitani
					vx_k = v_no_kalman.x;
					vy_k = v_no_kalman.y;
					roll_k = att_no_kalman.x * 180 / Math.PI;
					yaw_k = att_no_kalman.z * 180 / Math.PI;
					fw.write(vx_k + "," + vy_k + ",");
					fw.write(mat.group(59) + "," + -Double.parseDouble(mat.group(60)) + ",");
					fw.write(roll_k + "," + yaw_k + ",");
					fw.write(mat.group(62) + "," + -Double.parseDouble(mat.group(64)) + "\n");
				}
				cur = br.readLine();
			}
			fw.close();
		} catch (Exception ex) {
			System.out.println(cur);
			System.out.println(ex.getMessage());
		}
	}
	static Quaternion getNtoB(ThreeAxis gyro){
		//N座標系（GPS）→B座標系（IMU）の変換に用いる回転クォータニオンを取得
		//B.11.1
		Quaternion q = new Quaternion();
		q.w =  Math.cos(gyro.z/2) * Math.cos(gyro.y/2) * Math.cos(gyro.x/2) + Math.sin(gyro.z/2) * Math.sin(gyro.y/2) * Math.sin(gyro.x/2);
		q.x =  Math.cos(gyro.z/2) * Math.cos(gyro.y/2) * Math.sin(gyro.x/2) - Math.sin(gyro.z/2) * Math.sin(gyro.y/2) * Math.cos(gyro.x/2);
		q.y =  Math.cos(gyro.z/2) * Math.sin(gyro.y/2) * Math.cos(gyro.x/2) + Math.sin(gyro.z/2) * Math.cos(gyro.y/2) * Math.sin(gyro.x/2);
		q.z =  Math.sin(gyro.z/2) * Math.cos(gyro.y/2) * Math.cos(gyro.x/2) - Math.cos(gyro.z/2) * Math.sin(gyro.y/2) * Math.sin(gyro.x/2);
		return q;
	}
	static double fitPI(double a){
		//[-PI,PI]の範囲に当てはめる
		a = a % (2 * Math.PI);
		if(a > Math.PI){
			a -= 2 * Math.PI;
		}
		if(a <-Math.PI){
			a += 2 * Math.PI;
		}
		return a;
	}
}