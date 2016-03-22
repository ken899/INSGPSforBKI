package strapDown;

class StrapDown{
	private final double DeltaT = 0.01;//航法演算感覚
	private Motorcycle motorcycle = new Motorcycle();//推定される自動二輪車の状態
	private double[][] transformBtoC = new double[3][3];//座標軸Bから座標軸Cへの変換行列
	private double[][] transformEtoC = new double[3][3];//座標軸Eから座標軸Cへの変換行列
	private Quaternion quatenion = new Quaternion();//クォータニオン.B座標系からC座標系への回転クォータニオン
	private Quaternion quatenionEtoC = new Quaternion();//クォータニオン.E座標系からC座標系への回転クォータニオン
	private GravityModel g = new GravityModel();//地球の重力モデル.C座標系？
	private ThreeAxis AV = new ThreeAxis();//角速度.B座標系
	private ThreeAxis RAV = new ThreeAxis();//相対角速度.C座標系
	private ThreeAxis ERAV = new ThreeAxis();//地球自転角速度.C座標系
	private ThreeAxis earth_model = new ThreeAxis();//地球モデル.C座標系

	private int tmp;//デバッグ用
	//国総研式には高度の更新式がない．今回は常に0
	StrapDown(Motorcycle m){
		//初期化処理
		m.pos.x = m.pos.x / 180 * Math.PI;
		m.pos.y = m.pos.y / 180 * Math.PI;
		motorcycle = m;
//		wander_angle = getWanderAngle(m.pos);
		motorcycle.wander_angle = 0;

		//国総研式
		quatenion.w =  Math.cos(motorcycle.gyro_v.y/2) * Math.cos(motorcycle.gyro_v.x/2) ;
		quatenion.x =  Math.cos(motorcycle.gyro_v.y/2) * Math.sin(motorcycle.gyro_v.x/2) ;
		quatenion.y =  Math.sin(motorcycle.gyro_v.y/2) * Math.cos(motorcycle.gyro_v.x/2) ;
		quatenion.z = -Math.sin(motorcycle.gyro_v.y/2) * Math.sin(motorcycle.gyro_v.x/2) ;
		//成岡式
//		quatenion.w =  Math.cos(motorcycle.gyro.z/2) * Math.cos(motorcycle.gyro.y/2) * Math.cos(motorcycle.gyro.x/2) + Math.sin(motorcycle.gyro.z/2) * Math.sin(motorcycle.gyro.y/2) * Math.sin(motorcycle.gyro.x/2);
//		quatenion.x =  Math.cos(motorcycle.gyro.z/2) * Math.cos(motorcycle.gyro.y/2) * Math.sin(motorcycle.gyro.x/2) - Math.sin(motorcycle.gyro.z/2) * Math.sin(motorcycle.gyro.y/2) * Math.cos(motorcycle.gyro.x/2);
//		quatenion.y =  Math.cos(motorcycle.gyro.z/2) * Math.sin(motorcycle.gyro.y/2) * Math.cos(motorcycle.gyro.x/2) + Math.sin(motorcycle.gyro.z/2) * Math.cos(motorcycle.gyro.y/2) * Math.sin(motorcycle.gyro.x/2);
//		quatenion.z =  Math.sin(motorcycle.gyro.z/2) * Math.cos(motorcycle.gyro.y/2) * Math.cos(motorcycle.gyro.x/2) - Math.cos(motorcycle.gyro.z/2) * Math.sin(motorcycle.gyro.y/2) * Math.sin(motorcycle.gyro.x/2);
		transformBtoC = getTransformMatrix(quatenion);

		//国総研式
		transformEtoC[0][0] = -Math.cos(motorcycle.wander_angle) * Math.sin(motorcycle.pos.y) * Math.cos(motorcycle.pos.x) + Math.sin(motorcycle.wander_angle) * Math.sin(motorcycle.pos.x);
		transformEtoC[0][1] = -Math.cos(motorcycle.wander_angle) * Math.sin(motorcycle.pos.y) * Math.sin(motorcycle.pos.x) - Math.sin(motorcycle.wander_angle) * Math.cos(motorcycle.pos.x);
		transformEtoC[0][2] = Math.cos(motorcycle.wander_angle) * Math.cos(motorcycle.pos.y);
		transformEtoC[1][0] = Math.sin(motorcycle.wander_angle) * Math.sin(motorcycle.pos.y) * Math.cos(motorcycle.pos.x) + Math.cos(motorcycle.wander_angle) * Math.sin(motorcycle.pos.x);
		transformEtoC[1][1] = Math.sin(motorcycle.wander_angle) * Math.sin(motorcycle.pos.y) * Math.sin(motorcycle.pos.x) - Math.cos(motorcycle.wander_angle) * Math.sin(motorcycle.pos.x);
		transformEtoC[1][2] = -Math.sin(motorcycle.wander_angle) * Math.cos(motorcycle.pos.y);
		transformEtoC[2][0] = Math.cos(motorcycle.pos.y) * Math.cos(motorcycle.pos.x);
		transformEtoC[2][1] = Math.cos(motorcycle.pos.y) * Math.sin(motorcycle.pos.x);
		transformEtoC[2][2] = Math.sin(motorcycle.pos.y);
		//成岡式
//		quatenionEtoC.w = ( Math.cos((motorcycle.pos.y + motorcycle.wander_angle)/2) * (Math.cos(-motorcycle.pos.x/2)+Math.sin(-motorcycle.pos.x/2))) / Math.sqrt(2);
//		quatenionEtoC.x = ( Math.sin((motorcycle.pos.y - motorcycle.wander_angle)/2) * (Math.cos(-motorcycle.pos.x/2)-Math.sin(-motorcycle.pos.x/2))) / Math.sqrt(2);
//		quatenionEtoC.y = (-Math.cos((motorcycle.pos.y - motorcycle.wander_angle)/2) * (Math.cos(-motorcycle.pos.x/2)-Math.sin(-motorcycle.pos.x/2))) / Math.sqrt(2);
//		quatenionEtoC.z = ( Math.cos((motorcycle.pos.y + motorcycle.wander_angle)/2) * (Math.cos(-motorcycle.pos.x/2)+Math.sin(-motorcycle.pos.x/2))) / Math.sqrt(2);
//		transformEtoC = getTransformMatrix(quatenionEtoC);

		g = g.updateGravityModel(transformEtoC, motorcycle.pos.z);

		earth_model = getEarthModel(motorcycle);//地球モデルの計算
		RAV = getRelativeAngularVelocity(motorcycle,earth_model);//相対角速度算出
		ERAV = getEarthRotationAngularVelocity(motorcycle.pos.y,motorcycle.wander_angle);//地球自転角速度の算出
	}
	public Motorcycle test(IMU imu){
		//ストラップダウン方式による二輪車の姿勢位置算出
		tmp++;
//		if(tmp == 10){
//			System.exit(1);
//		}
//		motorcycle.pshowR();
		if(Double.isNaN(motorcycle.v.x)){
			System.out.println(tmp);
			System.exit(1);
		}
		AV = angularVelocityPreprocessing(transformBtoC,ERAV,RAV);//角速度前処理.自転と相対速度の影響量
		AV = angularVelocityProcessing(imu,AV);//角速度処理．
		quatenion = updateQuatenion(AV,quatenion);//クォータニオンの更新
		transformBtoC = getTransformMatrix(quatenion);//座標変換行列BtoCの更新
//		System.out.println("BtoC");
//		for(int i=0;i<transformBtoC.length;i++){
//			for(int j=0;j<transformBtoC[i].length;j++){
//				System.out.print(transformBtoC[i][j]+",");
//			}
//			System.out.println();
//		}
//		motorcycle.gyro = getAttitudeAngle(quatenion);//姿勢角の算出
		motorcycle.gyro_v = getAttitudeAngle(transformBtoC);//姿勢角の算出
//		motorcycle.acc = accBtoC(imu.acc,transformBtoC);//加速度変換処理.加速度をBからCへ.回転行列
		motorcycle.acc = quatenion.rotateFrame(quatenion, imu.acc);//加速度変換処理.加速度をBからCへ.クォータニオン
		motorcycle.v = getVelocity(motorcycle.v,motorcycle.acc,ERAV,RAV,g);//速度補正演算
		RAV = getRelativeAngularVelocity(motorcycle,earth_model);//相対角速度算出
		transformEtoC = updateEtoC(transformEtoC,RAV);//座標変換行列EtoCの更新
//		System.out.println("EtoC");
//		for(int i=0;i<transformEtoC.length;i++){
//			for(int j=0;j<transformEtoC[i].length;j++){
//				System.out.print(transformEtoC[i][j]+",");
//			}
//			System.out.println();
//		}
		motorcycle.pos = getPosition(transformEtoC);//位置，wander_angleの算出
		motorcycle.wander_angle = getWanderangle(transformEtoC);
		ERAV = getEarthRotationAngularVelocity(motorcycle.pos.y,motorcycle.wander_angle);//地球自転角速度の算出
		g = g.updateGravityModel(transformEtoC, motorcycle.pos.z);//地球重力モデルの計算
		earth_model = getEarthModel(motorcycle);//地球モデルの計算
		motorcycle.v.print();
		return motorcycle;
	}
	public Motorcycle main(IMU imu){
		//ストラップダウン方式による二輪車の姿勢位置算出
		AV = angularVelocityPreprocessing(transformBtoC,ERAV,RAV);//角速度前処理
		AV = angularVelocityProcessing(imu,AV);//角速度処理
		quatenion = updateQuatenion(AV,quatenion);//クォータニオンの更新
		transformBtoC = getTransformMatrix(quatenion);//座標変換行列BtoCの更新
		motorcycle.gyro_v = getAttitudeAngle(quatenion);//姿勢角の算出
		motorcycle.acc = accBtoC(imu.acc,transformBtoC);//加速度変換処理
		motorcycle.v = getVelocity(motorcycle.v,motorcycle.acc,ERAV,RAV,g);//速度補正演算
		RAV = getRelativeAngularVelocity(motorcycle,earth_model);//相対角速度算出
		transformEtoC = updateEtoC(transformEtoC,RAV);//座標変換行列EtoCの更新
		motorcycle.pos = getPosition(transformEtoC);//位置，wander_angleの算出
		ERAV = getEarthRotationAngularVelocity(motorcycle.pos.y,motorcycle.wander_angle);//地球自転角速度の算出
		g = g.updateGravityModel(transformEtoC, motorcycle.pos.z);//地球重力モデルの計算
		earth_model = getEarthModel(motorcycle);//地球モデルの計算
		return motorcycle;
	}
	//mine
//	private double getWanderAngle(ThreeAxis p){
//		//二輪車の位置する緯度経度からワンダーアングルを算出
//		//p.x:lon,p.y:lat
//		double theta,op,on,pa,oa,na,bp;
//		double r =  6356752.314140356;// 地球の極半径．m．wikiより
//		theta = p.y;
//		on = r;
//		na = on * Math.sin(theta);//2
//		oa = on * Math.cos(theta);//4
//		bp = 2 * r * theta / Math.PI;//6
//		op = bp / Math.sin(theta);//5
//		pa = op - oa;//3
//		return Math.atan( pa / na);//1
//	}
	//morioka
	private double getWanderAngle(ThreeAxis p,double plon,double wander_angle){
		wander_angle += (p.y - plon) * Math.sin(p.x);
		return wander_angle;
	}
	private ThreeAxis angularVelocityPreprocessing(double[][] BtoC,ThreeAxis ERAV,ThreeAxis RAV) {
		//角速度前処理
		//地球自転角速度，相対角速度による角速度に対する影響量を算出
		ThreeAxis AV = new ThreeAxis();
//		AV.x = (-ERAV.x -RAV.x) * DeltaT;
//		AV.y = (-ERAV.y -RAV.y) * DeltaT;
//		AV.z = (-ERAV.z -RAV.z) * DeltaT;
		AV.x = (-ERAV.x -RAV.x);
		AV.y = (-ERAV.y -RAV.y);
		AV.z = (-ERAV.z -RAV.z);
		AV = transform3x3MatrixT(BtoC,AV);
		return AV;
	}
	private ThreeAxis angularVelocityProcessing(IMU imu,ThreeAxis pAV) {
		//角速度処理
		//計測した角速度から地球自転角速度，相対角速度による影響量を引く
//		System.out.println(imu.gyro.x);
		AV.x = imu.gyro_v.x - pAV.x;
		AV.y = imu.gyro_v.y - pAV.y;
		AV.z = imu.gyro_v.z - pAV.z;
//		単調増加，減少するかのチェック
//		for(int i=0; i< 1;i++){
//			if(imu.gyro.x >0){
//				System.out.print("1,");
//			}else{
//				System.out.print("-1,");
//			}
//		}
//		System.out.println("");

		return AV;
	}
	private Quaternion updateQuatenion2(ThreeAxis AV,Quaternion q){
		double[][] Omega = new double[4][4];
		for(int i = 0; i < 4; i++){
			Omega[i][i] = 0;
		}
		Omega[0][1] =  AV.z/2;
		Omega[0][2] = -AV.y/2;
		Omega[0][3] =  AV.x/2;
		Omega[1][0] = -AV.z/2;
		Omega[1][2] =  AV.x/2;
		Omega[1][3] =  AV.y/2;
		Omega[2][0] =  AV.y/2;
		Omega[2][1] = -AV.x/2;
		Omega[2][3] =  AV.z/2;
		Omega[3][0] = -AV.x/2;
		Omega[3][1] = -AV.y/2;
		Omega[3][2] = -AV.z/2;
		double[] tmp = {1,1,1,1};
		tmp = transform4x4Matrix(Omega,tmp);//quatenionの時間微分
//		単調増加，減少するかのチェック
//		for(int i=0; i< 4;i++){
//			if(tmp[i] >0){
//				System.out.print("1,");
//			}else{
//				System.out.print("-1,");
//			}
//		}
//		System.out.println("");
		q.x += tmp[0] * DeltaT;
		q.y += tmp[1] * DeltaT;
		q.z += tmp[2] * DeltaT;
		q.w += tmp[3] * DeltaT;
		return q;
	}
	private Quaternion updateQuatenion(ThreeAxis AV,Quaternion q){
		Quaternion a = new Quaternion();
		a.x = AV.x / 2 * DeltaT;
		a.y = AV.y / 2 * DeltaT;
		a.z = AV.z / 2 * DeltaT;
		a = a.mul(a,q);
		q = q.plus(a);
		return q;
	}
	private double[][] getTransformMatrix(Quaternion q){
		double[][] t_matrix = new double [3][3];
		//国総研式
		t_matrix[0][0] = Math.pow(q.w,2) + Math.pow(q.x,2) - Math.pow(q.y,2) - Math.pow(q.z,2);
		t_matrix[0][1] = 2 * (q.x * q.y - q.w * q.z );
		t_matrix[0][2] = 2 * (q.x * q.z + q.w * q.y );
		t_matrix[1][0] = 2 * (q.x * q.y + q.w * q.z );
		t_matrix[1][1] = Math.pow(q.w,2) - Math.pow(q.x,2) + Math.pow(q.y,2) - Math.pow(q.z,2);
		t_matrix[1][2] = 2 * (q.y * q.z - q.w * q.x );
		t_matrix[2][0] = 2 * (q.x * q.z - q.w * q.y );
		t_matrix[2][1] = 2 * (q.y * q.z + q.w * q.x );
		t_matrix[2][2] = Math.pow(q.w,2) - Math.pow(q.x,2) - Math.pow(q.y,2) + Math.pow(q.z,2);
		//計算ノート式
//		t_matrix[0][0] = Math.pow(q.w,2) + Math.pow(q.x,2) - Math.pow(q.y,2) - Math.pow(q.z,2);
//		t_matrix[0][1] = 2 * (q.x * q.y + q.w * q.z );
//		t_matrix[0][2] = 2 * (q.x * q.z - q.w * q.y );
//		t_matrix[1][0] = 2 * (q.x * q.y - q.w * q.z );
//		t_matrix[1][1] = Math.pow(q.w,2) - Math.pow(q.x,2) + Math.pow(q.y,2) - Math.pow(q.z,2);
//		t_matrix[1][2] = 2 * (q.y * q.z + q.w * q.x );
//		t_matrix[2][0] = 2 * (q.x * q.z + q.w * q.y );
//		t_matrix[2][1] = 2 * (q.y * q.z - q.w * q.x );
//		t_matrix[2][2] = Math.pow(q.w,2) - Math.pow(q.x,2) - Math.pow(q.y,2) + Math.pow(q.z,2);
		return t_matrix;
	}

	private ThreeAxis getAttitudeAngle(Quaternion q){
		//姿勢角の算出
		ThreeAxis gyro = new ThreeAxis();//x:roll y:pitch z:yaw
		gyro.x = Math.atan( 2 * (q.y * q.z + q.w * q.x) / (Math.pow(q.w, 2) - Math.pow(q.x, 2) - Math.pow(q.y, 2) + Math.pow(q.z, 2)));
		gyro.y = Math.asin( 2 * (q.x * q.z - q.w * q.y));
		gyro.z = Math.atan(-2 * (q.x * q.y + q.w * q.z) / (Math.pow(q.w, 2) + Math.pow(q.x, 2) - Math.pow(q.y, 2) - Math.pow(q.z, 2)));
		return gyro;
	}
	private ThreeAxis getAttitudeAngle(double[][] BtoC){
		//姿勢角の算出
		ThreeAxis gyro = new ThreeAxis();//x:roll y:pitch z:yaw
		gyro.x = Math.atan( BtoC[2][1] / BtoC[2][2]);
		gyro.y = Math.asin( BtoC[2][0]);
		gyro.z = Math.atan(-BtoC[1][0] / BtoC[0][0]);
		return gyro;
	}

	private ThreeAxis accBtoC(ThreeAxis acc_b, double[][] BtoC){
		//加速度変換処理
		return transform3x3Matrix(BtoC,acc_b);
	}
	private ThreeAxis getVelocity(ThreeAxis v,ThreeAxis acc_c,ThreeAxis ERAV,ThreeAxis RAV,ThreeAxis g){
		//速度の算出．C座標系
		//それぞれRAV：遠心力，2*ERAV:コリオリ力 の係数
		ThreeAxis v_new = new ThreeAxis();
		v_new.x += (acc_c.x - (RAV.y + 2 * ERAV.y) * v.z + (RAV.z + 2 * ERAV.z) * v.y + g.x) * DeltaT;
		v_new.y += (acc_c.y - (RAV.z + 2 * ERAV.z) * v.x + (RAV.x + 2 * ERAV.x) * v.z + g.y) * DeltaT;
		v_new.z += (acc_c.z - (RAV.x + 2 * ERAV.x) * v.y + (RAV.y + 2 * ERAV.y) * v.x + g.z) * DeltaT;
		return v_new;
	}
	ThreeAxis getRelativeAngularVelocity(Motorcycle m,ThreeAxis earth_model){
		//相対角速度の算出
		ThreeAxis RAV = new ThreeAxis();//移動体の相対角速度
		RAV.x = -m.v.y / (earth_model.x + m.pos.z);
		RAV.y =  m.v.x / (earth_model.y + m.pos.z);
		RAV.z = 0;
		return RAV;
	}
	double[][] updateEtoC(double[][] EtoC,ThreeAxis RAV){
		//変換行列EtoCの更新
		double[][] newEtoC = new double[3][3];
		double[][] omega = {{     0,-RAV.z, RAV.y},
							{ RAV.z,     0,-RAV.x},
							{-RAV.y, RAV.x,     0}};
		//(I+omega*dt)*EtoC
		for(int i = 0;i < omega.length; i++){
			for(int j = 0;j < omega[i].length; j++){
				omega[i][j] = omega[i][j] * DeltaT;
			}
		}
		for(int i = 0;i < omega.length; i++){
			omega[i][i] += 1;
		}
		for(int i = 0;i < newEtoC.length; i++){
			for(int j = 0;j < newEtoC[i].length; j++){
				for(int k = 0;k < newEtoC[i].length; k++){
					newEtoC[i][j] += omega[i][k] * EtoC[k][j];
				}
			}
		}
		return newEtoC;
	}
	ThreeAxis getPosition(double[][] EtoC){
		//緯度，経度
		//p.x:経度，p.y:緯度
		//高度更新はなし
		ThreeAxis p = new ThreeAxis();
		p.x = Math.PI + Math.atan(EtoC[2][1]/EtoC[2][0]);
		p.y = Math.asin(EtoC[2][2]);
		return p;
	}
	double getWanderangle(double[][] EtoC){
		return Math.atan(-EtoC[1][2]/EtoC[0][2]);
	}
	ThreeAxis getEarthRotationAngularVelocity(double lat,double wander_angle){
		//地球自転角速度の算出
		double omega = 7.2921159 * Math.pow(10,-5);
		ThreeAxis ERAV = new ThreeAxis();
		ERAV.x = omega * Math.cos(lat) * Math.cos(wander_angle);
		ERAV.y =-omega * Math.cos(lat) * Math.sin(wander_angle);
		ERAV.z = omega * Math.sin(lat);
		return ERAV;
	}
	ThreeAxis getEarthModel(Motorcycle m){
		//地球モデルの計算．三軸を返り値にしているがx,yのみ使用
		//rx:地球の横の半径．ry:地球の縦の半径
		double a = 6378137;//地球の赤道面での長半径．m．wikiより
		double e = 0.081819191042815791;//離心率．wikiより
		ThreeAxis r = new ThreeAxis();
		double rm,rp;
		rm = a * (1 - Math.pow(e,2)) / Math.pow(1 - Math.pow(e,2) * Math.pow(Math.sin(m.pos.y),2) , 1.5);
		rp = a  / Math.sqrt(1 - Math.pow(e,2) * Math.pow(Math.sin(m.pos.y),2) );
		r.x = rp * rm * (1 + Math.pow(Math.tan(m.wander_angle), 2)) / (rp + rm *  Math.pow(Math.tan(m.wander_angle), 2));
		r.y = rp * rm * (1 + Math.pow(Math.tan(m.wander_angle), 2)) / (rm + rp *  Math.pow(Math.tan(m.wander_angle), 2));
		return r;
	}
	 ThreeAxis transform3x3Matrix(double[][] a, ThreeAxis b){
		//[3x3]行列a * [3x1]行列bの演算
		ThreeAxis c = new ThreeAxis();
		c.x = a[0][0] * b.x + a[0][1] * b.y + a[0][2] * b.z;
		c.y = a[1][0] * b.x + a[1][1] * b.y + a[1][2] * b.z;
		c.z = a[2][0] * b.x + a[2][1] * b.y + a[2][2] * b.z;
		return c;
	}
	 ThreeAxis transform3x3MatrixT(double[][] a, ThreeAxis b){
		//[3x3]行列aの転置 * [3x1]行列bの演算
		ThreeAxis c = new ThreeAxis();
		c.x = a[0][0] * b.x + a[1][0] * b.y + a[2][0] * b.z;
		c.y = a[0][1] * b.x + a[1][1] * b.y + a[2][1] * b.z;
		c.z = a[0][2] * b.x + a[1][2] * b.y + a[2][2] * b.z;
		return c;
	}
	 double[] transform4x4Matrix(double[][] a, double[] b){
		//[4*4]行列a * [4*1]行列bの演算
		double[] c = {0,0,0,0};
		for(int i = 0; i < 4; i++){
			for(int j = 0; j < 4; j++){
				c[i] += a[i][j] * b[j];
			}
		}
		return c;
	}
}
