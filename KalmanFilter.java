package com.chongqing.support.algorithm;
/**
 * 卡尔曼滤波
 */
import java.util.ArrayList;

public class KalmanFilter {
	
	private final double Q = 0.000001;
	private final double R = 0.001;
	private ArrayList<Double> dataArrayList;
	private int length;
	
	private double z[]; // data
	private double xhat[];
	private double xhatminus[];
	private double P[];
	private double Pminus[];
	private double K[];

	public KalmanFilter(ArrayList<Double> arrayList){
		this.dataArrayList = arrayList;
		this.length = arrayList.size();
		z = new double[length];
		xhat = new double[length];
		xhatminus = new double[length];
		P = new double[length];
		Pminus = new double[length];
		K = new double[length];		
		xhat[0] = 0;
		P[0] = 1.0;
		
		for (int i = 0; i<length; i++) {
			z[i] = (double)dataArrayList.get(i);
		}
	}
	
	public ArrayList<Double> calc(){
		if(dataArrayList.size()<2){
			return dataArrayList;
		}
		for(int k = 1; k < length; k++) {
		    // X(k|k-1) = AX(k-1|k-1) + BU(k) + W(k),A=1,BU(k) = 0  
		    xhatminus[k] = xhat[k-1];  

		    // P(k|k-1) = AP(k-1|k-1)A' + Q(k) ,A=1
		    Pminus[k] = P[k-1]+Q;        
		  
		    // Kg(k)=P(k|k-1)H'/[HP(k|k-1)H' + R],H=1
		    K[k] = Pminus[k]/( Pminus[k]+R);   
		    
		    // X(k|k) = X(k|k-1) + Kg(k)[Z(k) - HX(k|k-1)], H=1
		    xhat[k] = xhatminus[k]+K[k]*(z[k]-xhatminus[k]);   
		    
		    //P(k|k) = (1 - Kg(k)H)P(k|k-1), H=1
		    P[k] = (1-K[k])*Pminus[k] ;
		}
		
		for (int i = 0; i<length; i++){
			dataArrayList.set(i, xhat[i]);
		}
		dataArrayList.remove(0);
		return dataArrayList;
	}
	
}
