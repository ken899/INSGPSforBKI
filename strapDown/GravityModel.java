package strapDown;

class GravityModel extends ThreeAxis{
//	フィート
//	private final double G_0  = 32.087606;
//	private final double G_L1 = 0.1693891;
//	private final double G_L2 = 7.47483 * Math.pow(10,-4);
//	private final double G_H1 = 9.6227  * Math.pow(10,-8);
//	private final double G_H2 = 6.409   * Math.pow(10,-10);
//	private final double G_H3 = 6.8512  * Math.pow(10,-15);
//	private final double G_N  = 1.619   * Math.pow(10,-8);
//	メートル
	private final double G_0  = 32.087606 / 3.28083;
	private final double G_L1 = 0.1693891 / 3.28083;
	private final double G_L2 = 7.47483 * Math.pow(10,-4) / 3.28083;
	private final double G_H1 = 9.6227  * Math.pow(10,-8) / 3.28083;
	private final double G_H2 = 6.409   * Math.pow(10,-10) / 3.28083;
	private final double G_H3 = 6.8512  * Math.pow(10,-15) / 3.28083;
	private final double G_N  = 1.619   * Math.pow(10,-8) / 3.28083;
	GravityModel updateGravityModel(double[][] transformEtoC,double height){
		//height[ft]
		x = G_N * height * transformEtoC[2][2] * transformEtoC[0][2];
		y = G_N * height * transformEtoC[2][2] * transformEtoC[1][2];
		z = -(G_0 + G_L1 * Math.pow(transformEtoC[2][2],2) + G_L2 * Math.pow(transformEtoC[2][2],4))
				* (1 - (G_H1 - G_H2 * Math.pow(transformEtoC[2][2],2)) * height + G_H3 * Math.pow(height,2) );
		return this;
	}
}