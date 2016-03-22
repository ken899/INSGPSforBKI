package strapDown;

class StrapDownNaruoka{
	//成岡式．徹底してクォータニオンによる演算をする
	private final double DeltaT = 0.01;//航法演算間隔
	static Motorcycle motorcycle = new Motorcycle();//推定される自動二輪車の状態．g座標上の速度，角度，位置
	static Quaternion n2b = new Quaternion();//クォータニオン
	static Quaternion e2n = new Quaternion();//クォータニオン
	private Quaternion g2n = new Quaternion();//クォータニオン
	private Quaternion b2g = new Quaternion();//クォータニオン
	private ThreeAxis v_2e_4n = new ThreeAxis();//n座標上の速度
	//カルマンフィルターに渡すためグローバル
	static ThreeAxis omega_n_e2i = new ThreeAxis();
	static ThreeAxis omega_n_n2e = new ThreeAxis();
	static ThreeAxis r_n2e = new ThreeAxis();
	static ThreeAxis r_e2n = new ThreeAxis();
	static double omega_e2i;//地球の自転速度
	static double R_normal;
	static IMU imu = new IMU();
	static ThreeAxis g = new ThreeAxis();//g:n座標系における重力加速度
	static double R_e;

	StrapDownNaruoka(Motorcycle m){
		//初期状態
		m.pos.x = m.pos.x / 180 * Math.PI;
		m.pos.y = m.pos.y / 180 * Math.PI;
		m.gyro.z = m.gyro.z / 180 * Math.PI;
		motorcycle = m;

		Quaternion e2g = new Quaternion();
		e2g = getEtoG(m);
		g2n = getGtoN(m);
		e2n = getEtoN(e2g,g2n);
		n2b = getNtoB(m);
		n2b = n2b.conjugate();
		n2b = n2b.regularize();
		e2n = e2n.regularize();
		g2n = g2n.regularize();
		b2g = b2g.regularize();
	}
	public Motorcycle main(IMU imu){
		//ストラップダウン方式による二輪車の姿勢位置算出
		//imuの単位は加速度：m/s 角速度：rad/s
		ThreeAxis omega_e2i_4n = new ThreeAxis();
		ThreeAxis omega_n2e_4n = new ThreeAxis();//地球に対する回転をn座標系で見た値
		omega_e2i_4n = getOmega_e2i_4n(e2n);
		omega_n2e_4n = getOmega_n2e_4n(motorcycle);
		//速度推定
//		v_2e_4n = new ThreeAxis();
		v_2e_4n = updateV_2e_4n(v_2e_4n,n2b.conjugate(),imu.acc,omega_e2i_4n,omega_n2e_4n,motorcycle.pos);//n座標上の速度
//		v_2e_4n.show();
		motorcycle.v = getVerocity(v_2e_4n,motorcycle.wander_angle);//g座標上の速度
//		motorcycle.v = getVerocity(motorcycle,imu.acc,n2b.conjugate(),e2n,omega_e2i_4n,omega_n2e_4n);
		//e2n更新
		e2n = updateEtoN(omega_n2e_4n,e2n);
		e2n = e2n.regularize();
		//位置推定
		motorcycle.pos.y = getLat(e2n);
		motorcycle.pos.x = getLon(e2n);
		motorcycle.wander_angle = getWanderangle(e2n);
		//g2n更新
		g2n = getGtoN(motorcycle);
		g2n = g2n.regularize();
		//n2b更新
		n2b = updateNtoB(n2b, imu.gyro_v, omega_e2i_4n, omega_n2e_4n);
		n2b = n2b.regularize();
//		n2b = updateNtoB(n2b, imu.gyro_v, omega_e2i_4n, omega_n2e_4n);
//		b2g更新
		b2g = n2b.mul(n2b.conjugate(), g2n.conjugate());
		b2g = b2g.regularize();
		//姿勢推定
		motorcycle.gyro.x = getRoll(b2g);
		motorcycle.gyro.y = getPitch(b2g);
		motorcycle.gyro.z = getYaw(b2g);
		return motorcycle;
	}
	ThreeAxis updateV_2e_4n(ThreeAxis v_2e_4n,Quaternion b2n,ThreeAxis imu_acc,ThreeAxis omega_e2i_4n,ThreeAxis omega_n2e_4n,ThreeAxis pos){
		//速度計算2号．n座標系速度とg座標系速度に分割．これはn座標系部
		ThreeAxis d = new ThreeAxis();
		ThreeAxis tmp = new ThreeAxis();
		double a;
		d = b2n.rotateFrame(b2n, imu_acc);//imuの加速度をn座標系に変換．3.3.18の1番目の{}
//		d.z += getGravity(pos);//重力を考慮．3.3.18の2番目の{}
//		tmp = omega_e2i_4n.mul(2);//3.3.18の3番目の{}
//		tmp = tmp.plus(omega_n2e_4n);//3.3.18の3番目の{}
//		tmp = tmp.crossProduct(tmp, v_2e_4n);//3.3.18の3番目の{}
//		d = d.minus(tmp);//3.3.18の3番目の{}
//		double omega_e2i = 7.2921159 * Math.pow(10,-5);//自転速度.rad/s
//		Quaternion omega_e2i_4e = new Quaternion();
//		omega_e2i_4e.x = e2n.w*e2n.y + e2n.x*e2n.z;//3.3.37
//		omega_e2i_4e.y = e2n.z*e2n.y - e2n.x*e2n.w;//3.3.37
//		omega_e2i_4e = omega_e2i_4e.mul(2*omega_e2i*omega_e2i*(getRnormal(pos.x)+pos.z));//3.3.37
//		omega_e2i_4e = omega_e2i_4e.rotateFrame(e2n,omega_e2i_4e);//3.3.18の4番目の{}
//		d = d.minus(omega_e2i_4e);//3.3.18の4番目の{}
		d = d.mul(DeltaT);
		v_2e_4n = v_2e_4n.plus(d);
		return v_2e_4n;
	}
	ThreeAxis getVerocity(ThreeAxis v_2e_4n,double wander_angle){
		//速度計算2号．n座標系速度とg座標系速度に分割．これはg座標系部
		ThreeAxis v = new ThreeAxis();
		v.x = v_2e_4n.x * Math.sin(wander_angle) - v_2e_4n.y * Math.cos(wander_angle);
		v.y = v_2e_4n.x * Math.cos(wander_angle) - v_2e_4n.y * Math.sin(wander_angle);
		v.z = v_2e_4n.z;
		return v;
	}
	ThreeAxis getVerocity(Motorcycle m,ThreeAxis imu_acc,Quaternion b2n,Quaternion e2n,ThreeAxis omega_e2i_4n,ThreeAxis omega_n2e_4n){
		//速度計算．vはΔT前の速度．accはIMUの観測値.n座標系でしか考えてないからダメ
		ThreeAxis acc_n = new Quaternion();
		acc_n = b2n.rotateFrame(b2n, imu_acc);//3.3.18 1番目のq
		Quaternion g = new Quaternion();
		g.z = getGravity(m.pos);
		acc_n.plus(g);//3.3.18 2番目のq
		omega_e2i_4n = omega_e2i_4n.mul(2);//3.3.18  3番目の{}
		omega_e2i_4n = omega_e2i_4n.plus(omega_n2e_4n);//3.3.18  3番目のq
		omega_e2i_4n = omega_e2i_4n.crossProduct(omega_e2i_4n, m.v);//3.3.18  3番目のq
		acc_n = acc_n.minus(omega_e2i_4n);//3.3.18  3番目のq
		double omega_e2i = 7.2921159 * Math.pow(10,-5);//自転速度.rad/s
		Quaternion omega_e2i_4e = new Quaternion();
		omega_e2i_4e.x = e2n.w*e2n.y + e2n.x*e2n.z;//3.3.37
		omega_e2i_4e.y = e2n.z*e2n.y - e2n.x*e2n.w;//3.3.37
		omega_e2i_4e = omega_e2i_4e.mul(2*omega_e2i*omega_e2i*(getRnormal(m.pos.x)+m.pos.z));//3.3.37
		acc_n = acc_n.minus(omega_e2i_4e.rotateFrame(e2n,omega_e2i_4e));//3.3.18 4番目のq
		m.v = m.v.plus(acc_n.mul(DeltaT));
		return m.v;
	}
	ThreeAxis getOmega_n2e_4n(Motorcycle m){
		//n座標系の相対角速度
		Quaternion omega_n2e_4n = new Quaternion();
		Double omega_n2e_4g_0 = m.v.x / (getRnormal(m.pos.x) + m.pos.z);
		Double omega_n2e_4g_1 =-m.v.y / (getRmeridian(m) + m.pos.z);

		omega_n2e_4n.x = omega_n2e_4g_0 * Math.cos(m.wander_angle) + omega_n2e_4g_1 * Math.sin(m.wander_angle);
		omega_n2e_4n.y =-omega_n2e_4g_0 * Math.sin(m.wander_angle) + omega_n2e_4g_1 * Math.cos(m.wander_angle);
		return omega_n2e_4n;
	}
	ThreeAxis getOmega_n2e_4n2(Motorcycle m){
		Quaternion omega_n2e_4n = new Quaternion();
		omega_n2e_4n.x = m.v.y * (Math.pow(Math.cos(m.wander_angle),2)/(getRnormal(m.pos.x) + m.pos.z) + Math.pow(Math.sin(m.wander_angle),2)/(getRmeridian(m) + m.pos.z))
				+ m.v.x * Math.cos(m.wander_angle) * Math.sin(m.wander_angle) * (1/(getRnormal(m.pos.x) + m.pos.z) - 1/(getRmeridian(m) + m.pos.z));//3.3.35
		omega_n2e_4n.y =-m.v.x * (Math.pow(Math.cos(m.wander_angle),2)/(getRmeridian(m) + m.pos.z) + Math.pow(Math.sin(m.wander_angle),2)/(getRnormal(m.pos.x) + m.pos.z))
				+ m.v.y * Math.cos(m.wander_angle) * Math.sin(m.wander_angle) * (1/(getRmeridian(m) + m.pos.z) - 1/(getRnormal(m.pos.x) + m.pos.z));//3.3.35
		return omega_n2e_4n;
	}
	ThreeAxis getOmega_e2i_4n(Quaternion e2n){
		//n座標系における自転速度を求める
		Quaternion omega_e2i_4n = new Quaternion();
		Quaternion omega_e2i_4e = new Quaternion();
		double omega_ei = 7.2921159 * Math.pow(10,-5);//自転速度.rad/s
		omega_e2i_4e.z = omega_ei;//3.3.20
		omega_e2i_4n = e2n.rotateFrame(e2n, omega_e2i_4e);//3.3.19.n座標系における自転速度
		return omega_e2i_4n;
	}
	Quaternion updateEtoN(ThreeAxis omega_n2e_4n,Quaternion e2n){
		//3.3.38
		Quaternion q = new Quaternion();
		q = q.mul(e2n,omega_n2e_4n);
		q = q.mul(0.5);
		e2n = e2n.plus(q.mul(DeltaT));
		return e2n;
	}
	double getLat(Quaternion e2n){
		//B.10.3
		return Math.atan(e2n.z / e2n.w) - Math.atan(e2n.x / e2n.y);
	}
	double getLon(Quaternion e2n){
		//B.10.2
		//asinの範囲が[-PI/2,PI/2]，日本の経度はおおよそ138度あたりということでMath.PI-で補正
		return Math.PI - Math.asin(1 - 2 * (e2n.w*e2n.w + e2n.z*e2n.z));
	}
	double getWanderangle(Quaternion e2n){
		//B.10.4
		return Math.atan(e2n.z / e2n.w) + Math.atan(e2n.x / e2n.y);
	}
	Quaternion updateNtoB(Quaternion n2b,ThreeAxis imu_gyro, ThreeAxis omega_e2i_4n,ThreeAxis omega_n2e_4n){
		//3.3.43
		Quaternion dot_q_n2b = new Quaternion();
		Quaternion tmp = new Quaternion();
		dot_q_n2b = n2b.mul(n2b,imu_gyro);
		tmp = tmp.plus(omega_e2i_4n);
		tmp = tmp.plus(omega_n2e_4n);
		tmp = tmp.mul(tmp,n2b);
		dot_q_n2b = dot_q_n2b.minus(tmp);
		dot_q_n2b = dot_q_n2b.mul(0.5);
		n2b = n2b.plus(dot_q_n2b.mul(DeltaT));
		return n2b;
	}
	Quaternion updateNtoB2(Quaternion n2b,ThreeAxis imu_gyro, ThreeAxis omega_e2i_4n,ThreeAxis omega_n2e_4n){
		//3.3.43
		Quaternion dot_q_n2b = new Quaternion();
		omega_e2i_4n = omega_e2i_4n.plus(omega_n2e_4n);
		dot_q_n2b.assignmentThreeAxis(omega_e2i_4n);
		dot_q_n2b = dot_q_n2b.mul(dot_q_n2b,n2b);
		dot_q_n2b = dot_q_n2b.minus(n2b.mul(n2b,imu_gyro));
		dot_q_n2b = dot_q_n2b.mul(-0.5);
		n2b = n2b.plus(dot_q_n2b.mul(DeltaT));
		return n2b;
	}
	double updateWanderangle(ThreeAxis p,double plon,double wander_angle){
		//B.4.1
		wander_angle += ((p.y - plon) * Math.sin(p.x) *DeltaT);
		return wander_angle;
	}
	Quaternion diffItoE(Quaternion q){
		//ItoEのクォータニオンの時間微分
		//2007mt.pdf B.7より
		double omega = 7.2921159 * Math.pow(10,-5);//自転速度
		Quaternion tmp = new Quaternion();
		tmp.z = omega;
		q = q.mul(tmp, q);
		q = q.mul(0.5);
		return q;
	}
	Quaternion getEtoG(Motorcycle m){
		//EtoGのクォータニオンの値
		//B.8.4
		Quaternion q = new Quaternion();
		q.w = Math.cos(m.pos.y / 2) * (Math.cos(-m.pos.x / 2) + Math.sin(-m.pos.x / 2));
		q.x = Math.sin(m.pos.y / 2) * (Math.cos(-m.pos.x / 2) - Math.sin(-m.pos.x / 2));
		q.y =-Math.cos(m.pos.y / 2) * (Math.cos(-m.pos.x / 2) - Math.sin(-m.pos.x / 2));
		q.z = Math.sin(m.pos.y / 2) * (Math.cos(-m.pos.x / 2) + Math.sin(-m.pos.x / 2));
		q = q.mul(1/Math.sqrt(2));
		return q;
	}
	Quaternion getGtoN(Motorcycle m){
		//GtoNのクォータニオンの値
		//B.9.1
		Quaternion q = new Quaternion();
		q.w = Math.cos(m.wander_angle / 2);
		q.z = Math.sin(m.wander_angle / 2);
		return q;
	}
	Quaternion getEtoN(Quaternion e2g,Quaternion g2n){
		//B.10.1
		Quaternion q = new Quaternion();
		q = q.mul(e2g, g2n);
		return q;
	}
	Quaternion getNtoB(Motorcycle m){
		//B.11.1
		Quaternion q = new Quaternion();
		q.w =  Math.cos(motorcycle.gyro.z/2) * Math.cos(motorcycle.gyro.y/2) * Math.cos(motorcycle.gyro.x/2) + Math.sin(motorcycle.gyro.z/2) * Math.sin(motorcycle.gyro.y/2) * Math.sin(motorcycle.gyro.x/2);
		q.x =  Math.cos(motorcycle.gyro.z/2) * Math.cos(motorcycle.gyro.y/2) * Math.sin(motorcycle.gyro.x/2) - Math.sin(motorcycle.gyro.z/2) * Math.sin(motorcycle.gyro.y/2) * Math.cos(motorcycle.gyro.x/2);
		q.y =  Math.cos(motorcycle.gyro.z/2) * Math.sin(motorcycle.gyro.y/2) * Math.cos(motorcycle.gyro.x/2) + Math.sin(motorcycle.gyro.z/2) * Math.cos(motorcycle.gyro.y/2) * Math.sin(motorcycle.gyro.x/2);
		q.z =  Math.sin(motorcycle.gyro.z/2) * Math.cos(motorcycle.gyro.y/2) * Math.cos(motorcycle.gyro.x/2) - Math.cos(motorcycle.gyro.z/2) * Math.sin(motorcycle.gyro.y/2) * Math.sin(motorcycle.gyro.x/2);
		return q;
	}
	double getYaw(Quaternion b2g){
		//B.11.2
		return Math.atan(2 * (b2g.x * b2g.y + b2g.w * b2g.z) / (Math.pow(b2g.w, 2) + Math.pow(b2g.x, 2) - Math.pow(b2g.y, 2) - Math.pow(b2g.z, 2)));
	}
	double getPitch(Quaternion b2g){
		//B.11.3
		return Math.asin( 2 * (b2g.w * b2g.y - b2g.x * b2g.z));
	}
	double getRoll(Quaternion b2g){
		//B.11.4
		return Math.atan( 2 * (b2g.y * b2g.z + b2g.w * b2g.x) / (Math.pow(b2g.w, 2) - Math.pow(b2g.x, 2) - Math.pow(b2g.y, 2) + Math.pow(b2g.z, 2)));
	}
	double getRmeridian(Motorcycle m){
		//C.2.3
		//南北方向曲率半径
		double e = 0.0818191908426;//離心率
		double re = 6378137;//赤道半径
		return re * (1 - e*e)/Math.pow(1 - e*e * Math.sin(m.pos.x)*Math.sin(m.pos.x), 1.5);
	}
	double getRnormal(double lon){
		//C.2.5
		//東西方向曲率半径
		double e = 0.0818191908426;//離心率
		double re = 6378137;//赤道半径
		return re /Math.sqrt(1 - e*e * Math.sin(lon)*Math.sin(lon));
	}
	double getGravity(ThreeAxis pos){
		//C.4.1
		double gwgs0 = 9.7803267714;
		double gwgs1 = 0.00193185138639;
		double e = 0.0818191908426;
		return gwgs0 * (1 + gwgs1 * Math.sin(pos.y)*Math.sin(pos.y)) / Math.sqrt(1 - e*e * Math.sin(pos.y)*Math.sin(pos.y));
	}
}
