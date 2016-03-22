package strapDown;


import jama.Matrix;

class QuaternionTest{
	//b2n座標変換のクォータニオンがうまく仕事するかどうかのテスト
	//容易に作成でき間違えないであろうオイラ―角，行列による座標変換と結果を比較する
	public static void main(String[] args){
		Quaternion b2n = new Quaternion();//b2nの座標編kンクォータニオン
		//yaw->pitch->rollの順で回転させてb座標をn座標へ一致させる
		//x->y->zの順で回転させるとb2nのクォータニオンと同じ動作ができるはず
		ThreeAxis angle = new ThreeAxis();//三軸角度
		ThreeAxis gyro = new ThreeAxis();//変換対象の角度．名前はgyroだけど多分時間にとらわれず何点か変換比べるだけ

		Matrix euler = new Matrix(3,3);//jkalmanについてきたjamaを使ってみる
		Matrix angle_matrix = new Matrix(3,1);
		Matrix gyro_matrix = new Matrix(3,1);

		ThreeAxis res_quaternion = new ThreeAxis();
		ThreeAxis res_euler = new ThreeAxis();

		//set data
		//現在の姿勢
		angle.x = 15 * Math.PI / 180;//roll
		angle.y = 0 * Math.PI / 180;//pitch
		angle.z = 93 * Math.PI / 180;//yaw
		//センサが取得した値
		gyro.x  =-4 * Math.PI / 180;//roll
		gyro.y  = 0 * Math.PI / 180;//pitch
		gyro.z  = 1 * Math.PI / 180;//yaw

		//x:theta y:psi z:phi
		euler.set(0, 0, Math.cos(angle.y)*Math.cos(angle.z));
		euler.set(0, 1,-Math.cos(angle.y)*Math.sin(angle.z));
		euler.set(0, 2, Math.sin(angle.y));
		euler.set(1, 0, Math.sin(angle.x)*Math.sin(angle.y)*Math.cos(angle.z) + Math.cos(angle.x)*Math.sin(angle.z));
		euler.set(1, 1,-Math.sin(angle.x)*Math.sin(angle.y)*Math.sin(angle.z) + Math.cos(angle.x)*Math.cos(angle.z));
		euler.set(1, 2,-Math.sin(angle.x)*Math.cos(angle.y));
		euler.set(2, 0,-Math.cos(angle.x)*Math.sin(angle.y)*Math.cos(angle.z) + Math.sin(angle.x)*Math.sin(angle.z));
		euler.set(2, 1, Math.cos(angle.x)*Math.sin(angle.y)*Math.sin(angle.z) + Math.sin(angle.x)*Math.cos(angle.z));
		euler.set(2, 2, Math.cos(angle.x)*Math.cos(angle.y));

		angle_matrix.set(0,0,angle.x);
		angle_matrix.set(1,0,angle.y);
		angle_matrix.set(2,0,angle.z);
		gyro_matrix.set(0,0,gyro.x);
		gyro_matrix.set(1,0,gyro.y);
		gyro_matrix.set(2,0,gyro.z);

		Matrix tmp = new Matrix(3,1);
		tmp = euler.times(gyro_matrix);
		res_euler.x = tmp.get(0, 0);
		res_euler.y = tmp.get(1, 0);
		res_euler.z = tmp.get(2, 0);

		b2n = getNtoB(angle).conjugate();
		b2n.print();
		res_quaternion = b2n.rotateFrame(gyro);
		res_quaternion.showDeg();
		System.out.println("---");
		angle.x = -angle.x;
		b2n = getNtoB(angle).conjugate();
		b2n.print();
		res_quaternion = b2n.rotateFrame(gyro);
		res_quaternion.showDeg();
		System.out.println("---");


		res_quaternion.showDeg();
//		res_euler.showDeg();

		double roll;
		if(true){
			roll = Math.acos(res_quaternion.z/gyro.z)/Math.PI*180;
		}else{
			roll = 360 - Math.acos(res_quaternion.z/gyro.z)/Math.PI*180;
		}

		System.out.println(res_quaternion.z + "," + gyro.z);

	}
	static Quaternion getNtoB(ThreeAxis angle){
		//B.11.1
		Quaternion q = new Quaternion();
		q.w =  Math.cos(angle.z/2) * Math.cos(angle.y/2) * Math.cos(angle.x/2) + Math.sin(angle.z/2) * Math.sin(angle.y/2) * Math.sin(angle.x/2);
		q.x =  Math.cos(angle.z/2) * Math.cos(angle.y/2) * Math.sin(angle.x/2) - Math.sin(angle.z/2) * Math.sin(angle.y/2) * Math.cos(angle.x/2);
		q.y =  Math.cos(angle.z/2) * Math.sin(angle.y/2) * Math.cos(angle.x/2) + Math.sin(angle.z/2) * Math.cos(angle.y/2) * Math.sin(angle.x/2);
		q.z =  Math.sin(angle.z/2) * Math.cos(angle.y/2) * Math.cos(angle.x/2) - Math.cos(angle.z/2) * Math.sin(angle.y/2) * Math.sin(angle.x/2);
		return q;
	}
}