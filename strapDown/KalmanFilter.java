package strapDown;

class KalmanFilter{
	//状態行列：[v e2n h n2b b] 要素数18
	//	v:速度 要素数3
	//	e2n:クォータニオンe2n 要素数4
	//	h:高度 要素数1
	//	n2b:クォータニオンn2b 要素数4
	//	b:零点 要素数6 加速度計ジャイロの角三軸のオフセット誤差
	//入力行列：[a omega_b2i_4b g db] 要素数15
	//	a:加速度 要素数3
	//	omega_b2i_4b:角速度 要素数3
	//	g:重力 要素数1
	//	db:零点ドリフト 要素数6
	//P:誤差分散行列 16x16
	// 微小クォータニオンを微小三次元ベクトルによって表現する．dq = [1 d]
	// クォータニオンは2つなので18-(4-3)*2=16
	//Z:観測行列
	//	v:速度 要素数3
	//	e2n:クォータニオン 要素数4
	//	h:高度 要素数1
	Matrix matrix = new Matrix();
	static double[][] x = new double[16][1];//状態行列
	static double[][] z = new double[8][1];//観測
	static double[][] p = new double[16][16];//誤差共分散行列
	static double[][] q = new double[12][12];//IMUノイズ．システムノイズ
	static double[][] r = new double[12][12];//GPSノイズ．観測ノイズ

	static final double DT = 0.05;
	StrapDownNaruoka s;
	KalmanFilter(){
		//初期化

	}

	StrapDownNaruoka main(StrapDownNaruoka s2){
		s = s2;
		Motorcycle motorcycle = s.motorcycle;
		//システム：motorcycle
		//観測：gps
		timeUpdate();
		measurementUpdate();
		transMatrix2Structure();
		return s;
	}
	void transMatrix2Structure(){
		s.motorcycle.v.x = x[0][0];
		s.motorcycle.v.y = x[1][0];
		s.motorcycle.v.z = x[2][0];
		s.e2n.w = x[2][0];
		s.e2n.x = x[3][0];
		s.e2n.y = x[4][0];
		s.e2n.z = x[5][0];
		s.motorcycle.pos.z = x[6][0];
		s.n2b.w = x[7][0];
		s.n2b.x =-x[8][0];
		s.n2b.y =-x[9][0];
		s.n2b.z =-x[10][0];
	}
	void timeUpdate(){
		//時間更新
		//状態行列，誤差共分散行列を変化させる
		//	状態行列の変化の大部分はStrapDownNaruoka中で行う．
		//	零点は(4.2.5)よりホワイトノイズ

		//誤差共分散行列Pの更新
		//phi,ganma:遷移行列
		//Aは10x10の正方行列だが12x12の正方行列として扱う．増えた行と列は0の値のみで構成される
		//phi = (I + Adt);
		double[][] a = new double[10][10];
		a = getA();
		double[][] identity = new double[10][10];
		for(int i = 0; i < 10; i++){
			identity[i][i] = 1;
		}
		double[][] phi = new double[10][10];
		for(int i = 0; i < 10; i++){
			for(int j = 0; j < 10; j++){
				phi[i][j] = identity[i][j] + a[i][j] * DT;
			}
		}

		//Bは12x9の行列
		//ganma = Bdt;
		double[][] b = new double[12][9];
		b = getB();
		double[][] ganma = new double[12][9];
		for(int i = 0; i < 12; i++){
			for(int j = 0; j < 9; j++){
				ganma[i][j] = b[i][j] * DT;
			}
		}
		//p = phi * p * phi^T + ganma * q * ganma^T
		//p:誤差共分散行列．初期値決めてあとは流れで
		//q:IMUのノイズ．定数
		double[][] tmp;
		tmp = matrix.mul(phi,p);
		tmp = matrix.mul(tmp,matrix.trans(phi));
		double[][] tmp2;
		tmp2 = matrix.mul(ganma, q);
		tmp2 = matrix.mul(tmp2, matrix.trans(ganma));
		p = matrix.plus(tmp, tmp2);
	}
	void measurementUpdate(){
		//z:gps.v q_e2n h
		//gps dummy
		//観測更新
		double[][] hd = new double[10][16];
		for(int i = 0; i< 10; i++){
			hd[i][i] = 1;
		}
		hd[4][3] =-s.e2n.x;
		hd[4][4] =-s.e2n.y;
		hd[4][5] =-s.e2n.z;
		hd[5][3] = s.e2n.w;
		hd[5][4] = s.e2n.z;
		hd[5][5] =-s.e2n.y;
		hd[6][3] =-s.e2n.z;
		hd[6][4] = s.e2n.w;
		hd[6][5] = s.e2n.x;
		hd[7][3] = s.e2n.y;
		hd[7][4] =-s.e2n.x;
		hd[7][5] = s.e2n.w;

		double[][] k = new double[16][16];
		double[][] tmp = new double[16][16];
		tmp = matrix.mul(p, matrix.trans(hd));
		double[][] tmp2 = new double[16][16];
		tmp2 = matrix.mul(hd, p);
		tmp2 = matrix.mul(tmp2, matrix.trans(hd));
		tmp2 = matrix.plus(tmp2, r);
		k = matrix.mul(tmp, matrix.trans(tmp2));

		//p<-(I-K*Hd)P
		double[][] identity = new double[16][16];
		for(int i = 0; i < 16; i++){
			identity[i][i] = 1;
		}
		tmp = matrix.mul(k, hd);
		tmp = matrix.minus(identity,tmp);
		p = matrix.mul(tmp, p);

		//dx=K(z-hd*x)
		//x <- x-dx
		tmp = matrix.mul(hd, x);
		tmp = matrix.minus(z, tmp);
		tmp = matrix.mul(k, tmp);
		x = matrix.minus(x,tmp);
	}

	double[][] getA(){
		double[][] a = new double[10][10];
		double[][] aaa = new double[12][12];
		double[][][][] aa = new double[4][4][3][3];
		aa[0][0] = a00();
		aa[0][1] = a01();
		aa[0][2] = a02();
		aa[0][3] = a03();
		aa[1][0] = a10();
//		aa[1][1] = new double[3][3];
		aa[1][2] = a12();
//		aa[1][3] = new double[3][3];
		aa[2][0][2][0] = -1;
//		aa[2][1] = new double[3][3];
//		aa[2][2] = new double[3][3];
//		aa[2][3] = new double[3][3];
		aa[3][0] = a30();
		aa[3][1] = a31();
		aa[3][2] = a32();
		aa[3][3] = a33();
		for(int i = 0; i < 4; i++){
			for(int j = 0; j < 4; j++){
				for(int k = 0; k < 3; k++){
					for(int l = 0; l < 3; l++){
						aaa[i*3+k][j*3+l] = aa[i][j][k][l];
					}

				}
			}

		}
		int ai = 0;
		int aj = 0;
		for(int i = 0; i < 12; i++){
			aj = 0;
			for(int j = 0; j < 12; j++){
				a[ai][aj] = aaa[i][j];
				if(j != 7 && j != 8)aj++;
			}
			if(i != 7 && i != 8)ai++;
		}
		return a;
	}
	double[][] getB(){
		double[][] b = new double[12][9];
		double[][][][] bb = new double[4][3][3][3];
		bb[0][0] = s.n2b.getDCM();
		bb[0][2][0][0] = s.g.x;
		bb[0][2][1][0] = s.g.y;
		bb[0][2][2][0] = s.g.z;
		bb[3][1] = b31();
		for(int i = 0; i < 4; i++){
			for(int j = 0; j < 3; j++){
				for(int k = 0; k < 3; k++){
					for(int l = 0; l < 3; l++){
						b[i*3+k][j*3+l] = bb[i][j][k][l];
					}

				}
			}
		}
		return b;
	}
	double[][] a00(){
		double[][] a_00 = new double[3][3];
		double rh = 1 / (s.R_e + s.motorcycle.pos.z);
		ThreeAxis omega = new ThreeAxis();
		omega = s.omega_n_e2i;
		omega.mul(2);
		omega.plus(s.omega_n_n2e);
		a_00[0][0] = rh * s.r_n2e.z;
		a_00[0][1] = omega.z;
		a_00[0][2] =-omega.y;
		a_00[1][0] =-omega.z;
		a_00[1][1] = rh * s.r_n2e.z;
		a_00[1][2] = omega.x;
		a_00[2][0] = omega.y + rh * -s.r_n2e.x;
		a_00[2][1] = omega.x + rh * -s.r_n2e.y;
		a_00[2][2] = 0;
		return a_00;
	}
	double[][] a01(){
		//b.1.13
		double[][] a_01 = new double[3][3];
		double[][] dcm_e2n = s.e2n.getDCM();
		a_01[0][0] = dcm_e2n[2][1] * s.r_n2e.y - dcm_e2n[1][1] * s.r_n2e.z;
		a_01[0][1] = dcm_e2n[1][0] * s.r_n2e.z - dcm_e2n[2][0] * s.r_n2e.y;
		a_01[1][0] = dcm_e2n[0][1] * s.r_n2e.z - dcm_e2n[2][1] * s.r_n2e.x;
		a_01[1][1] = dcm_e2n[2][0] * s.r_n2e.x - dcm_e2n[0][0] * s.r_n2e.z;
		a_01[2][0] = dcm_e2n[1][1] * s.r_n2e.x - dcm_e2n[0][1] * s.r_n2e.y;
		a_01[2][1] = dcm_e2n[0][0] * s.r_n2e.y - dcm_e2n[1][0] * s.r_n2e.x;
		double rh = 2 * s.omega_e2i * s.omega_e2i * (s.R_normal+s.motorcycle.pos.z);
		a_01[0][0] += rh * ( dcm_e2n[0][0]*dcm_e2n[1][0] + dcm_e2n[0][1]*dcm_e2n[2][2] + dcm_e2n[0][2]*dcm_e2n[2][1]);
		a_01[0][1] += rh * (-dcm_e2n[0][0]*dcm_e2n[2][2] + dcm_e2n[0][1]*dcm_e2n[1][0] - dcm_e2n[0][2]*dcm_e2n[2][0]);
		a_01[1][0] += rh * ( dcm_e2n[1][0]*dcm_e2n[1][0] + dcm_e2n[1][1]*dcm_e2n[2][2] + dcm_e2n[1][2]*dcm_e2n[2][1]);
		a_01[1][1] += rh * (-dcm_e2n[1][0]*dcm_e2n[2][2] - dcm_e2n[1][1]*dcm_e2n[1][0] - dcm_e2n[1][2]*dcm_e2n[2][0]);
		a_01[2][0] += rh * ( dcm_e2n[2][0]*dcm_e2n[1][0] + dcm_e2n[2][1]*dcm_e2n[2][2] + dcm_e2n[2][2]*dcm_e2n[2][1]);
		a_01[2][1] += rh * (-dcm_e2n[2][0]*dcm_e2n[2][2] - dcm_e2n[2][1]*dcm_e2n[1][0] - dcm_e2n[2][2]*dcm_e2n[2][0]);
		return a_01;
	}
	double[][] a02(){
		double[][] a_02 = new double[3][3];
		double rh = - 1 / (s.R_e + s.motorcycle.pos.z) * (s.R_e + s.motorcycle.pos.z);
		a_02[0][0] = rh * s.r_n2e.x * s.r_n2e.z;
		a_02[1][0] = rh * s.r_n2e.y * s.r_n2e.z;
		a_02[2][0] = rh * ( -s.r_n2e.x * s.r_n2e.x - s.r_n2e.y * s.r_n2e.y);
		double[][] dcm_e2n = s.e2n.getDCM();
		a_02[0][0] += s.omega_e2i * s.omega_e2i * (dcm_e2n[0][0] * dcm_e2n[2][0] - dcm_e2n[0][1] *dcm_e2n[2][1]);
		a_02[1][0] += s.omega_e2i * s.omega_e2i * (dcm_e2n[1][0] * dcm_e2n[2][0] - dcm_e2n[1][1] *dcm_e2n[2][1]);
		a_02[2][0] += s.omega_e2i * s.omega_e2i * (dcm_e2n[2][0] * dcm_e2n[2][0] - dcm_e2n[2][1] *dcm_e2n[2][1]);
		return a_02;
	}
	double[][] a03(){
		double[][] a_03 = new double[3][3];
		double[][] dcm_e2n = s.e2n.getDCM();
		//a_bは加速度の値
		a_03[0][0] = 0;
		a_03[0][1] = -2 * (-dcm_e2n[0][2]*s.imu.acc.x - dcm_e2n[1][2]*s.imu.acc.y - dcm_e2n[2][2]*s.imu.acc.z);
		a_03[0][2] = -2 * ( dcm_e2n[0][1]*s.imu.acc.x + dcm_e2n[1][1]*s.imu.acc.y + dcm_e2n[2][1]*s.imu.acc.z);
		a_03[1][0] = -2 * ( dcm_e2n[0][2]*s.imu.acc.x + dcm_e2n[1][2]*s.imu.acc.y + dcm_e2n[2][2]*s.imu.acc.z);
		a_03[1][1] = 0;
		a_03[1][2] = -2 * (-dcm_e2n[0][0]*s.imu.acc.x - dcm_e2n[1][0]*s.imu.acc.y - dcm_e2n[2][0]*s.imu.acc.z);
		a_03[2][0] = -2 * (-dcm_e2n[0][1]*s.imu.acc.x - dcm_e2n[1][1]*s.imu.acc.y - dcm_e2n[2][1]*s.imu.acc.z);
		a_03[2][1] = -2 * ( dcm_e2n[0][0]*s.imu.acc.x + dcm_e2n[1][0]*s.imu.acc.y + dcm_e2n[2][0]*s.imu.acc.z);
		a_03[2][2] = 0;
		return a_03;
	}
	double[][] a10(){
		double[][] a_10 = new double[3][3];
		double[][] dcm_e2n = s.e2n.getDCM();
		double rh = 1/(2 * (s.R_e + s.motorcycle.pos.z));
		a_10[0][0] = rh * -dcm_e2n[1][0];
		a_10[0][1] = rh *  dcm_e2n[0][0];
		a_10[1][0] = rh * -dcm_e2n[1][1];
		a_10[1][1] = rh *  dcm_e2n[0][1];
		a_10[2][0] = rh * -dcm_e2n[1][2];
		a_10[2][1] = rh *  dcm_e2n[0][2];
		return a_10;
	}
	double[][] a12(){
		double[][] a_12 = new double[3][3];
		double[][] dcm_e2n = s.e2n.getDCM();
		double rh = 1/(2 * (s.R_e + s.motorcycle.pos.z) * (s.R_e + s.motorcycle.pos.z));
		a_12[0][0] = rh *( dcm_e2n[1][0] * s.r_e2n.x - dcm_e2n[0][0] * s.r_e2n.y);
		a_12[1][0] = rh *( dcm_e2n[1][1] * s.r_e2n.x - dcm_e2n[0][1] * s.r_e2n.y);
		a_12[2][0] = rh *( dcm_e2n[1][2] * s.r_e2n.x - dcm_e2n[0][2] * s.r_e2n.y);
		return a_12;
	}
	double[][] a30(){
		double[][] a_30 = new double[3][3];
		double rh = 1 / (2 * (s.R_e + s.motorcycle.pos.z));
		a_30[0][1] = rh;
		a_30[1][0] =-rh;
		return a_30;
	}
	double[][] a31(){
		double[][] a_31 = new double[3][3];
		double[][] dcm_e2n = s.e2n.getDCM();
		a_31[0][0] = -s.omega_e2i * dcm_e2n[0][1];
		a_31[0][1] =  s.omega_e2i * dcm_e2n[0][0];
		a_31[1][0] = -s.omega_e2i * dcm_e2n[1][1];
		a_31[1][1] =  s.omega_e2i * dcm_e2n[1][0];
		a_31[2][0] = -s.omega_e2i * dcm_e2n[2][1];
		a_31[2][1] =  s.omega_e2i * dcm_e2n[2][0];
		return a_31;
	}
	double[][] a32(){
		double[][] a_32 = new double[3][3];
		double rh = 1/(2 * (s.R_e + s.motorcycle.pos.z) * (s.R_e + s.motorcycle.pos.z));
		a_32[0][0] = rh * s.r_e2n.y;
		a_32[1][0] = rh *-s.r_e2n.x;
		return a_32;
	}
	double[][] a33(){
		double[][] a_33 = new double[3][3];
		ThreeAxis tmp = s.omega_n_e2i;
		tmp.plus(s.omega_n_n2e);
		a_33[0][0] = 0;
		a_33[0][1] = tmp.z;
		a_33[0][2] =-tmp.y;
		a_33[1][0] =-tmp.z;
		a_33[1][1] = 0;
		a_33[1][2] = tmp.x;
		a_33[2][0] = tmp.y;
		a_33[2][1] =-tmp.x;
		a_33[2][2] = 0;
		return a_33;
	}
	double[][] b31(){
		double[][] b_31 = s.n2b.getDCM();
		for(int i = 0; i < b_31.length; i++){
			for(int j = 0; j < b_31[i].length; j++){
				b_31[i][j] /= 2;
			}
		}
		return b_31;
	}
}

class Matrix{
	double[][] plus(double[][] a, double[][] b){
		double[][] c;
		int w,h;
		if(a.length > b.length)
			h = a.length;
		else
			h = b.length;
		if(a[0].length > b[0].length)
			w = a[0].length;
		else
			w = b[0].length;
		c = new double[h][w];
		double[][] aa = new double[h][w];
		double[][] bb = new double[h][w];
		for(int i = 0; i < a.length; i++){
			for(int j = 0; j < a[0].length; j++){
				aa[i][j] = a[i][j];
			}
		}
		for(int i = 0; i < b.length; i++){
			for(int j = 0; j < b[0].length; j++){
				bb[i][j] = b[i][j];
			}
		}
		for(int i = 0; i < h; i++){
			for(int j = 0; j < w; j++){
				c[i][j] = aa[i][j] + bb[i][j];
			}
		}
		return c;
	}
	double[][] minus(double[][] a, double[][] b){
		double[][] c;
		int w,h;
		if(a.length > b.length)
			h = a.length;
		else
			h = b.length;
		if(a[0].length > b[0].length)
			w = a[0].length;
		else
			w = b[0].length;
		c = new double[h][w];
		double[][] aa = new double[h][w];
		double[][] bb = new double[h][w];
		for(int i = 0; i < a.length; i++){
			for(int j = 0; j < a[0].length; j++){
				aa[i][j] = a[i][j];
			}
		}
		for(int i = 0; i < b.length; i++){
			for(int j = 0; j < b[0].length; j++){
				bb[i][j] = b[i][j];
			}
		}
		for(int i = 0; i < h; i++){
			for(int j = 0; j < w; j++){
				c[i][j] = aa[i][j] - bb[i][j];
			}
		}
		return c;
	}
	double[][] mul(double[][] a, double[][] b){
		//行列の積
		double[][] a2,b2;
		int l;//計算範囲
		if(a[0].length > b.length)
			l = a[0].length;
		else
			l = b.length;

		a2 = new double [a.length][l];
		b2 = new double [l][b[0].length];

		for(int i = 0; i < a.length; i++){
			for(int j = 0; j < a[0].length; j++){
				a2[i][j] = a[i][j];
			}
		}
		for(int i = 0; i < b.length; i++){
			for(int j = 0; j < b[0].length; j++){
				b2[i][j] = b[i][j];
			}
		}

		double[][] c = new double[a.length][b[0].length];
		for(int i = 0; i < a.length; i++){
			for(int j = 0; j < b[0].length; j++){
				for(int k = 0; k < l; k++){
					c[i][j] += a2[i][k] * b2[k][j];
				}
			}
		}
		return c;
	}
	double[][] trans(double[][] a){
		//逆行列
		double[][] b = new double[a[0].length][a.length];
		for(int i = 0; i < b.length; i++){
			for(int j = 0; j < b[i].length; j++){
				b[i][j] = a[j][i];
			}

		}
		return b;
	}
}
