public static double[] MatrixToYawPitchRoll( Mat A )
{
        double[] angle = new double[3];
        angle[1] = -Math.asin( A.at<double>(3,1) );  //Pitch

        //Gymbal lock: pitch = -90
        if( A.at<double>(3,1)   r31 == 1 ){
            angle[0] = 0.0;             //yaw = 0
            angle[2] = Math.atan2( -A.at<double>(1,2), -A.at<double>(1,3) );    //Roll
//            System.out.println("Gimbal lock: pitch = -90");
        }

        //Gymbal lock: pitch = 90
        else if( A.at<double>(3,1) == -1 ){
            angle[0] = 0.0;             //yaw = 0
            angle[2] = Math.atan2( A.at<double>(1,2), A.at<double>(1,3) );    //Roll
//            System.out.println("Gimbal lock: pitch = 90");
        }
        //General solution
        else{
            angle[0] = Math.atan2(  A.at<double>(2,1), A.at<double>(1,1) );
            angle[2] = Math.atan2(  A.at<double>(3,2), A.at<double>(3,3) );
//            System.out.println("No gimbal lock");
        }
        return angle;   //Euler angles in order yaw, pitch, roll
}
