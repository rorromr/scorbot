
using namespace cv;

int mostrar_info = 1; // 1=SI | 0=NO
int cont_parche = 0;
int cont_parche_dump = 0;

// Se crea OCTREE con datos de PCL de entrada
float resolution_in = 512.0f;
pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree (resolution_in);
pcl::PointCloud<pcl::PointXYZ> pcl_kinectXYZ;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered2 (new pcl::PointCloud<pcl::PointXYZ>);

Mat ceros = (Mat_<double>(3,1));
int makers_on = 1; //1 = muestra markers
int proxy_state = 1; // 1=FREE_MOTION | 2=CONTACT | 3=ENTRENCHMENT

// AJUSTE DE PARAMETROS-------------------------------!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//double d_k = 0.050; //STEP SIZE PROXY MOVEMENT  0.025
double d_k = 0.01;
//double K_spring = 20.0;
//double K_spring = 30.0;
double K_spring = 80.0;//70
double K_damping = 70.0;
double F_const = 0.0;
double radius_R1 = 0.01; // radius_R1  0.12 // 0.15
double radius_R2 = 0.04; // radius_R2  0.19 // 0.28
double radius_R3 = 0.10; // radius_R3  0.30 // 0.35

/*
double pose_phantom_actual_x, pose_phantom_actual_y, pose_phantom_actual_z, pose_phantom_actual_time, delta_pose_x, delta_pose_y, delta_pose_z;
double pose_phantom_ant_x = 0.0, pose_phantom_ant_y = 0.0, pose_phantom_ant_z = 0.0, pose_phantom_ant_time = 0.0;
double mag_delta_pose, delta_tiempo,velocidad_HIP;
*/

Mat norm_vector = (Mat_<double>(3,1));
Mat u_k = (Mat_<double>(3,1));
Mat u_k_hat = (Mat_<double>(3,1));
Mat u_kp = (Mat_<double>(3,1));
Mat u_kp_hat = (Mat_<double>(3,1));
Mat Force_HIP = (Mat_<double>(3,1));
Mat Force_HIP_dump = (Mat_<double>(3,1));
double sk_x, sk_y, sk_z, u_k_norm,u_k_norm_old, u_k_vel, u_kp_norm, ang_norm_u_k, ang_proy_u_k;
Mat velocidad_HIP = (Mat_<double>(3,1));

pcl::PointXYZ searchPoint;
pcl::PointXYZ HIP_position;
pcl::PointXYZ proxy_position;
Mat HIP_position_mat = (Mat_<double>(3,1));
Mat proxy_position_mat = (Mat_<double>(3,1));
//searchPoint.x = 0.0;
//searchPoint.y = 0.0;
//searchPoint.z = 0.0;

double dist_euclid(Mat p, Mat q){
  double sum = 0.0;
  for(int i=0; i<p.rows; i++)
    sum += pow(p.at<double>(i,0)-q.at<double>(i,0),2);
  double resultado = sqrt(sum);
  return resultado;
}

double dot_product(Mat p, Mat q){
  double sum = 0.0;
  for(int i=0; i<p.rows; i++)
    sum += p.at<double>(i,0)*q.at<double>(i,0);
  double resultado = sum;
  return resultado;
}

double norm(Mat x){
  double resultado = sqrt(dot_product(x,x));
  return resultado;
}

Mat normalize(Mat x){
  Mat mat_out = (Mat_<double>(x.rows,1));
  for(int i=0; i<x.rows; i++)
    mat_out.at<double>(i,0) = (x.at<double>(i,0)/norm(x));
  return mat_out;
}

Mat project_onto_plane(Mat x, Mat n){
  double d = dot_product(x , n) / norm(n);
  Mat mat_out = (Mat_<double>(n.rows,1));
  Mat p = (Mat_<double>(n.rows,1));
  Mat normalize_n =  normalize(n);
  for(int i=0; i<n.rows; i++)
    p.at<double>(i,0) = (d * normalize_n.at<double>(i,0));
  for(int j=0; j<n.rows; j++)
    mat_out.at<double>(j,0) = (x.at<double>(j,0)-p.at<double>(j,0));
  return mat_out;
}


void effectorPoseCallback(const geometry_msgs::PoseStamped & pose_marker){ //SOLO CUANDO HIP SE MUEVE!!!
  //MALA FUERZA - BUENA POSICION DE PROXY
  HIP_position.x = (double)(pose_marker.pose.position.x);
  HIP_position.y = (double)(pose_marker.pose.position.y);
  HIP_position.z = (double)(pose_marker.pose.position.z);
  //BUENA FUERZA - MALA POSICION DE PROXY
  //HIP_position.x = (double)(pose_marker.pose.position.z);
  //HIP_position.y = (double)(pose_marker.pose.position.x);
  //HIP_position.z = (double)(pose_marker.pose.position.y);

  HIP_position_mat.at<double>(0,0) = HIP_position.x;
  HIP_position_mat.at<double>(1,0) = HIP_position.y;
  HIP_position_mat.at<double>(2,0) = HIP_position.z;
}

//void pcl_kinect_Callback(const sensor_msgs::PointCloud2 & pcl_kinect){
void pcl_kinect_Callback(const sensor_msgs::PointCloud2ConstPtr & pcl_kinect){
  //void cloud_msg (const sensor_msgs::PointCloud2ConstPtr &msg)


  //pcl::PointCloud<pcl::PointXYZ> pcl_kinectXYZ;
  //pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_kinectXYZ_ptr (new pcl::PointCloud<pcl::PointXYZ>);
  //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered2 (new pcl::PointCloud<pcl::PointXYZ>);

  //pcl::fromROSMsg(pcl_kinect, pcl_kinectXYZ);
  pcl::PCLPointCloud2 pcl_aux;
  //pcl_conversions::toPCL(pcl_kinect, pcl_aux);
  pcl_conversions::toPCL(*pcl_kinect, pcl_aux);
  pcl::fromPCLPointCloud2 (pcl_aux, *cloud_filtered2);
  //pcl::fromPCLPointCloud2 (pcl_kinect, pcl_kinectXYZ);
  //*cloud_filtered2+=pcl_kinectXYZ;

  //pcl_kinectXYZ = pcl_conversions::toPCL(pcl_kinect);

  octree.deleteTree();
  octree.setInputCloud(cloud_filtered2);
  octree.addPointsFromInputCloud();
}

void proxy_movement(int proxy_state, Mat norm_vector){ // 1=FREE_MOTION | 2=CONTACT | 3=ENTRENCHMENT
  Mat norm_vector_aux = (Mat_<double>(3,1));
  u_k = proxy_position_mat-HIP_position_mat;
  u_k_norm_old = u_k_norm;
  u_k_norm = dist_euclid(HIP_position_mat,proxy_position_mat);
  u_k_vel = u_k_norm-u_k_norm_old;
  //std::cerr << "u_k_vel = " << u_k_vel << std::endl;


  if(fabs(u_k_norm) < 0.0001){ // Evita division por cero
    u_k_norm = 1.0;
    u_k_hat = 0.0;
  }
    //OJO BORRAR
  else{
    u_k_hat = u_k/u_k_norm;
  }
  //u_k_hat = u_k/u_k_norm;


  // 21 Enero 2016
  ang_norm_u_k = (acos(( (u_k_hat.at<double>(0,0)*norm_vector.at<double>(0,0)) + (u_k_hat.at<double>(1,0)*norm_vector.at<double>(1,0)) + (u_k_hat.at<double>(2,0)*norm_vector.at<double>(2,0)))/(1*1)) )*180/M_PI; //En grados
  //ang_proy_u_k = 90 - ang_norm_u_k;

  ////u_kp = (u_k_hat+norm_vector)/2;
  u_kp = -1.0*(u_k_hat+norm_vector)/2.0; //OJO: Borrar 21 Enero 2016 ANTIGUO (funciona bien)
  //Project u_k on the plane defined by norm_vector (denote this vector u_kp)
  //u_kp  = project_onto_plane(u_k, norm_vector); // 21 Enero 2016 NUEVO /(normal queda bloqueada en algunos puntos)

/*
    u_kp_norm = dist_euclid(u_kp,ceros);
    if(fabs(u_kp_norm) < 0.0001){ // Evita division por cero
        u_kp_norm = 1.0;
        u_kp_hat = 0.0;
    }
    //OJO BORRAR
    else{
        u_kp_hat = u_kp/u_kp_norm;
    }
    //u_kp_hat = u_kp/u_kp_norm;

*/
  u_kp_hat = normalize(u_kp);



  //std::cerr << "ang_norm_u_k = " << ang_norm_u_k << std::endl;

  if(proxy_state == 1){ // FREE_MOTION
    if(mostrar_info==1)
      std::cerr << "proxy_state = FREE_MOTION ----------------------------------------------------"  << std::endl;
    //ROS_INFO("proxy_state = FREE_MOTION ----------------------------------------------------");
    sk_x = d_k*(u_k_hat.at<double>(0,0));
    sk_y = d_k*(u_k_hat.at<double>(1,0));
    sk_z = d_k*(u_k_hat.at<double>(2,0));
    norm_vector_aux = norm_vector;
  }
  else if(proxy_state == 2 && fabs(ang_norm_u_k) >= 90){ // CONTACT and HIP outside the surface
    if(mostrar_info==1)
      std::cerr << "proxy_state = CONTACT and HIP outside the surface ----------------------------"  << std::endl;
    sk_x = d_k*(u_k_hat.at<double>(0,0));
    sk_y = d_k*(u_k_hat.at<double>(1,0));
    sk_z = d_k*(u_k_hat.at<double>(2,0));
    norm_vector_aux = norm_vector;
  }
  else if(proxy_state == 2 && fabs(ang_norm_u_k) < 90){ // CONTACT and HIP is inside the surface
    if(mostrar_info==1)
      std::cerr << "proxy_state = CONTACT and HIP is inside the surface --------------------------"  << std::endl;
    sk_x = d_k*(u_kp_hat.at<double>(0,0));
    sk_y = d_k*(u_kp_hat.at<double>(1,0));
    sk_z = d_k*(u_kp_hat.at<double>(2,0));
    norm_vector_aux = norm_vector;
  }
  else if(proxy_state == 3){ // ENTRENCHMENT
    if(mostrar_info==1)
      std::cerr << "proxy_state = ENTRENCHMENT ---------------------------------------------------"  << std::endl;
    sk_x = (d_k*norm_vector_aux.at<double>(0,0))/2.0; // Se usa un norm_vector_aux para que en el borde no se exista intercambio de normales <-||->
    sk_y = (d_k*norm_vector_aux.at<double>(1,0))/2.0; // Se usa un norm_vector_aux para que en el borde no se exista intercambio de normales <-||->
    sk_z = (d_k*norm_vector_aux.at<double>(2,0))/2.0; // Se usa un norm_vector_aux para que en el borde no se exista intercambio de normales <-||->


  }

  if(mostrar_info==1)
    //std::cerr << "u_k.at<double>(0,0) = " << u_k.at<double>(0,0) << "| u_k.at<double>(1,0) = " << u_k.at<double>(1,0) << " | u_k.at<double>(2,0) = " << u_k.at<double>(2,0) <<std::endl;
    //std::cerr << "sk_x = " << sk_x << "| sk_y = " << sk_y << " | sk_z = " << sk_z <<std::endl;
    std::cerr << "proxy_state = " << proxy_state <<std::endl;

  if( (fabs(u_k.at<double>(0,0))>0.025 || fabs(u_k.at<double>(1,0))>0.025 || fabs(u_k.at<double>(2,0))>0.025) && (proxy_state == 1 || proxy_state == 3) ){
    //if( proxy_state == 1 || proxy_state == 3){ // 21 Enero 2016
    if(mostrar_info==1)
      std::cerr << " SI ENTRA IF 1 " <<std::endl;
    proxy_position.x = proxy_position.x - sk_x;
    proxy_position.y = proxy_position.y - sk_y;
    proxy_position.z = proxy_position.z - sk_z;


    proxy_position_mat.at<double>(0,0) = proxy_position.x;
    proxy_position_mat.at<double>(1,0) = proxy_position.y;
    proxy_position_mat.at<double>(2,0) = proxy_position.z;

    searchPoint.x = proxy_position.x;
    searchPoint.y = proxy_position.y;
    searchPoint.z = proxy_position.z;

  }
  else if( fabs(ang_norm_u_k) > 8.0 && proxy_state == 2 ){
    //else if( proxy_state == 2 ){ //21 Enero 2016
    if(mostrar_info==1)
      std::cerr << " SI ENTRA IF 2 " <<std::endl;
    proxy_position.x = proxy_position.x - sk_x;
    proxy_position.y = proxy_position.y - sk_y;
    proxy_position.z = proxy_position.z - sk_z;
    proxy_position_mat.at<double>(0,0) = proxy_position.x;
    proxy_position_mat.at<double>(1,0) = proxy_position.y;
    proxy_position_mat.at<double>(2,0) = proxy_position.z;

    searchPoint.x = proxy_position.x;
    searchPoint.y = proxy_position.y;
    searchPoint.z = proxy_position.z;
  }
  else if(proxy_state == 4){
    proxy_position.x = HIP_position.x;
    proxy_position.y = HIP_position.y;
    proxy_position.z = HIP_position.z;

    proxy_position_mat.at<double>(0,0) = proxy_position.x;
    proxy_position_mat.at<double>(1,0) = proxy_position.y;
    proxy_position_mat.at<double>(2,0) = proxy_position.z;

    searchPoint.x = proxy_position.x;
    searchPoint.y = proxy_position.y;
    searchPoint.z = proxy_position.z;
  }
  else{ // NO SE ACTUALIZA POSICION DEL PROXY
    if(mostrar_info==1)
      std::cerr << " NO ENTRA IF" <<std::endl;
  }

}// FIN proxy_movement

void calc_force(int proxy_state){
  // CALCULA FUERZA!
  if(proxy_state == 1){ // FREE_MOTION
    Force_HIP.at<double>(0,0) = 0.0;
    Force_HIP.at<double>(1,0) = 0.0;
    Force_HIP.at<double>(2,0) = 0.0;
    Force_HIP_dump.at<double>(0,0) = 0.0;
    Force_HIP_dump.at<double>(1,0) = 0.0;
    Force_HIP_dump.at<double>(2,0) = 0.0;
  }
  else if(proxy_state == 2 && fabs(ang_norm_u_k) >= 90){ // CONTACT and HIP outside the surface
    Force_HIP.at<double>(0,0) = 0.0;
    Force_HIP.at<double>(1,0) = 0.0;
    Force_HIP.at<double>(2,0) = 0.0;
    Force_HIP_dump.at<double>(0,0) = 0.0;
    Force_HIP_dump.at<double>(1,0) = 0.0;
    Force_HIP_dump.at<double>(2,0) = 0.0;
  }
    //else if(proxy_state == 2 && fabs(ang_norm_u_k) < 90){ // CONTACT and HIP is inside the surface
  else if(proxy_state == 2 && fabs(ang_norm_u_k) < 90 && u_k_norm>((radius_R1+radius_R2)/2.0) ){ // CONTACT and HIP is inside the surface
    //Force_HIP = -1.0*u_k_hat*K_spring*(u_k_norm-((radius_R1+radius_R2)/2) );
    //Force_HIP = -1.0*u_k_hat*K_spring;
    //Force_HIP = -1.0*u_k_hat*K_spring*u_k_norm;  // TODO: Agregar k_d = Damping constant ORIGINAL
    //Force_HIP = -1.0*u_k_hat*K_spring*(u_k_norm-((radius_R1+radius_R2)/2.0) );
    Force_HIP = -1.0*u_k_hat*K_spring*(u_k_norm-((radius_R1+radius_R2)/2.0) ) -1.0*u_k_hat*F_const;
    Force_HIP_dump = -1.0*u_k_hat*K_spring*(u_k_norm-((radius_R1+radius_R2)/2.0) ) + (K_damping*(velocidad_HIP));  // TODO: Agregar k_d = Damping constant
    //Force_HIP = -1.0*u_k_hat*K_spring*u_k_norm - (K_damping*(u_k_norm-u_k_norm_old));
  }
    //else if(proxy_state == 3){ // ENTRENCHMENT
  else if(proxy_state == 3 && u_k_norm>((radius_R1+radius_R2)/2.0) ){ // ENTRENCHMENT
    //Force_HIP = -1.0*u_k_hat*K_spring*(u_k_norm-((radius_R1+radius_R2)/2) );
    //Force_HIP = -1.0*u_k_hat*K_spring;
    //Force_HIP = -1.0*u_k_hat*K_spring*u_k_norm;  // TODO: Agregar k_d = Damping constant  ORIGINAL
    //Force_HIP = -1.0*u_k_hat*K_spring*(u_k_norm-((radius_R1+radius_R2)/2.0) );
    Force_HIP = -1.0*u_k_hat*K_spring*(u_k_norm-((radius_R1+radius_R2)/2.0) ) -1.0*u_k_hat*F_const;
    Force_HIP_dump = -1.0*u_k_hat*K_spring*(u_k_norm-((radius_R1+radius_R2)/2.0) ) + (K_damping*(velocidad_HIP));  // TODO: Agregar k_d = Damping constant
    //Force_HIP = -1.0*u_k_hat*K_spring*u_k_norm - (K_damping*(u_k_norm-u_k_norm_old));
  }
  else{
    Force_HIP.at<double>(0,0) = 0.0;
    Force_HIP.at<double>(1,0) = 0.0;
    Force_HIP.at<double>(2,0) = 0.0;
    Force_HIP_dump.at<double>(0,0) = 0.0;
    Force_HIP_dump.at<double>(1,0) = 0.0;
    Force_HIP_dump.at<double>(2,0) = 0.0;

  }
  if(mostrar_info==1)
    //std::cerr << "u_k_hat = " << u_k_hat <<std::endl;
    std::cerr << "Force_HIP = \n" << Force_HIP <<std::endl;
}// FIN calc_force

Mat normal_vector(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::vector<int> index_points_R3){
  Mat normal = (Mat_<double>(3,1));

  if(index_points_R3.size() != 0){
    Mat proxy_position_vect = (Mat_<double>(3,1));
    proxy_position_vect.at<double>(0,0) = proxy_position.x;
    proxy_position_vect.at<double>(1,0) = proxy_position.y;
    proxy_position_vect.at<double>(2,0) = proxy_position.z;

    //Mat points_within_R3 = (Mat_<double>(3,index_points_R3.size()));
    Mat points_within_R3_i = (Mat_<double>(3,1));
    Mat sum_normal = (Mat_<double>(3,1));
    //Mat ceros = (Mat_<double>(3,1));
    Mat resta, resta_norm;
    double dist_2;
    ceros.at<double>(0,0) =  0.0; // X
    ceros.at<double>(1,0) =  0.0; // Y
    ceros.at<double>(2,0) =  0.0;// Z
    sum_normal.at<double>(0,0) =  0.0; // X
    sum_normal.at<double>(1,0) =  0.0; // Y
    sum_normal.at<double>(2,0) =  0.0;// Z

    //if(index_points_R3.size()>=4){ //EVITA OUTLIERS (solo cuando hay 3 o mas puntos en la PC => calcula la normal)
    for (int i = 0; i < index_points_R3.size (); i++){
      points_within_R3_i.at<double>(0,0) =  cloud->points[index_points_R3[i]].x; // X
      points_within_R3_i.at<double>(1,0) =  cloud->points[index_points_R3[i]].y; // Y
      points_within_R3_i.at<double>(2,0) =  cloud->points[index_points_R3[i]].z;// Z
      resta = proxy_position_vect-points_within_R3_i;
      //std::cerr << "  resta_x=  "<< resta.at<double>(0,0)<< " resta_y="<< resta.at<double>(1,0) << " resta_z="<< resta.at<double>(2,0) << std::endl;
      dist_2 = dist_euclid(proxy_position_vect,points_within_R3_i);
      resta_norm = resta/dist_2;
      sum_normal=sum_normal+resta_norm;
      //std::cerr << "  resta_norm_x=  "<< resta_norm.at<double>(0,0)<< " resta_norm_y="<< resta_norm.at<double>(1,0) << " resta_norm_z="<< resta_norm.at<double>(2,0) << std::endl;
    }
    //}
    double aux = dist_euclid(sum_normal,ceros);
    if(fabs(aux) < 0.00001){ // Evita division por cero
      aux = 1.0;
    }
    normal = sum_normal/aux;
  }

  else{
    normal.at<double>(0,0) = 0.0;
    normal.at<double>(1,0) = 0.0;
    normal.at<double>(2,0) = 0.0;
  }
  return normal;
}

void calc_proxy_state(std::vector<int> pointIdxRadiusSearch_R1,std::vector<int> pointIdxRadiusSearch_R2,std::vector<int> pointIdxRadiusSearch_R3){
  ///// proxy_state
  if(pointIdxRadiusSearch_R3.size() == 0){
    proxy_state = 4; // POINT CLOUD FREE
  }
  else if(pointIdxRadiusSearch_R2.size() == 0){
    proxy_state = 1; //free_motion
    //std::cerr << "proxy_state = free_motion -------------------------------------"  << std::endl;
  }
  else if(pointIdxRadiusSearch_R2.size() != 0 && pointIdxRadiusSearch_R1.size() == 0){
    //else if(pointIdxRadiusSearch_R2.size() >= 3 && pointIdxRadiusSearch_R1.size() == 0){
    proxy_state = 2; // contact
    //std::cerr << "proxy_state = contact -----------------------------------------"  << std::endl;
  }
  else if(pointIdxRadiusSearch_R1.size() != 0){
    //else if(pointIdxRadiusSearch_R1.size() >= 3){
    proxy_state = 3; // entrenchment
    //std::cerr << "proxy_state = entrenchment -----------------------------------"  << std::endl;
  }
  else{
    if(mostrar_info==1)
      std::cerr << "proxy_state =  ?? --------------------------------------------"  << std::endl;
  }
}

geometry_msgs::WrenchStamped force_outliers_filter(geometry_msgs::WrenchStamped fuerza_anterior, Mat fuerza_nueva){
  geometry_msgs::WrenchStamped force_filtered_out;
  if(fuerza_nueva.at<double>(0,0)==0.0 && fuerza_nueva.at<double>(1,0)==0.0 && fuerza_nueva.at<double>(2,0)==0.0 && fabs(fuerza_anterior.wrench.force.x)>0.01 && fabs(fuerza_anterior.wrench.force.y)>0.01 && fabs(fuerza_anterior.wrench.force.z)>0.01 && cont_parche<4) {
    force_filtered_out.wrench.force.x = fuerza_anterior.wrench.force.x;
    force_filtered_out.wrench.force.y = fuerza_anterior.wrench.force.y;
    force_filtered_out.wrench.force.z = fuerza_anterior.wrench.force.z;
    cont_parche++;
  }
  else{
    cont_parche=0;
    force_filtered_out.wrench.force.x = -fuerza_nueva.at<double>(0,0);
    force_filtered_out.wrench.force.y = -fuerza_nueva.at<double>(1,0);
    force_filtered_out.wrench.force.z = -fuerza_nueva.at<double>(2,0);
  }
  force_filtered_out.header.frame_id = "/link_0"; // OJO!!! REVISAR!!
  force_filtered_out.header.seq++;
  force_filtered_out.header.stamp=ros::Time::now();
  return force_filtered_out;
}

geometry_msgs::WrenchStamped force_outliers_filter_dump(geometry_msgs::WrenchStamped fuerza_anterior, Mat fuerza_nueva){
  geometry_msgs::WrenchStamped force_filtered_out;
  if(fuerza_nueva.at<double>(0,0)==0.0 && fuerza_nueva.at<double>(1,0)==0.0 && fuerza_nueva.at<double>(2,0)==0.0 && fabs(fuerza_anterior.wrench.force.x)>0.01 && fabs(fuerza_anterior.wrench.force.y)>0.01 && fabs(fuerza_anterior.wrench.force.z)>0.01 && cont_parche_dump<4) {
    force_filtered_out.wrench.force.x = fuerza_anterior.wrench.force.x;
    force_filtered_out.wrench.force.y = fuerza_anterior.wrench.force.y;
    force_filtered_out.wrench.force.z = fuerza_anterior.wrench.force.z;
    cont_parche_dump++;
  }
  else{
    cont_parche_dump=0;
    force_filtered_out.wrench.force.x = -fuerza_nueva.at<double>(0,0);
    force_filtered_out.wrench.force.y = -fuerza_nueva.at<double>(1,0);
    force_filtered_out.wrench.force.z = -fuerza_nueva.at<double>(2,0);
  }
  force_filtered_out.header.frame_id = "/link_0"; // OJO!!! REVISAR!!
  force_filtered_out.header.seq++;
  force_filtered_out.header.stamp=ros::Time::now();
  return force_filtered_out;
}

void pose_vel_Callback(const geometry_msgs::Vector3Stamped & msg){
  velocidad_HIP.at<double>(0,0) = msg.vector.x;
  velocidad_HIP.at<double>(1,0) = msg.vector.y;
  velocidad_HIP.at<double>(2,0) = msg.vector.z;
  //std::cerr << "velocidad_HIP = " << velocidad_HIP << std::endl;
}

/*
void pose_phantom_Callback(const geometry_msgs::PoseStamped & pose_phantom_in){
    pose_phantom_actual_x = pose_phantom_in.pose.position.x;
    pose_phantom_actual_y = pose_phantom_in.pose.position.y;
    pose_phantom_actual_z = pose_phantom_in.pose.position.z;
    //pose_phantom_actual_time = pose_phantom_in.header.stamp.nsec;
    pose_phantom_actual_time = pose_phantom_in.header.stamp.toSec();
    delta_pose_x = pose_phantom_actual_x - pose_phantom_ant_x;
    delta_pose_y = pose_phantom_actual_y - pose_phantom_ant_y;
    delta_pose_z = pose_phantom_actual_z - pose_phantom_ant_z;
    //std::cerr << "delta_pose_x = " << delta_pose_x << std::endl;
    Mat delta_pose = (Mat_<double>(3,1));
    delta_pose.at<double>(0,0) =  delta_pose_x; // X
    delta_pose.at<double>(1,0) =  delta_pose_y; // Y
    delta_pose.at<double>(2,0) =  delta_pose_z; // Z
    mag_delta_pose=dist_euclid(delta_pose, ceros);
    delta_tiempo = pose_phantom_actual_time-pose_phantom_ant_time;
    if(delta_tiempo != 0.0){
        velocidad_HIP = mag_delta_pose/delta_tiempo;
    }
    //velocidad_HIP = mag_delta_pose;
    //std::cerr << "velocidad_HIP = " << velocidad_HIP << std::endl;
    //std::cerr << "delta_tiempo = " << delta_tiempo << std::endl;
    pose_phantom_ant_x = pose_phantom_actual_x;
    pose_phantom_ant_y = pose_phantom_actual_y;
    pose_phantom_ant_z = pose_phantom_actual_z;
    pose_phantom_ant_time = pose_phantom_actual_time;
}
*/

int main2 (int argc, char** argv){
  std::vector<int> pointIdxRadiusSearch_R1;
  std::vector<float> pointRadiusSquaredDistance_R1;
  std::vector<int> pointIdxRadiusSearch_R2;
  std::vector<float> pointRadiusSquaredDistance_R2;
  std::vector<int> pointIdxRadiusSearch_R3;
  std::vector<float> pointRadiusSquaredDistance_R3;

  ceros.at<double>(0,0) =  0.0; // X
  ceros.at<double>(1,0) =  0.0; // Y
  ceros.at<double>(2,0) =  0.0; // Z

  velocidad_HIP.at<double>(0,0) = 0.0;
  velocidad_HIP.at<double>(1,0) = 0.0;
  velocidad_HIP.at<double>(2,0) = 0.0;

  ros::init(argc, argv, "pcd_example_node");
  ros::NodeHandle n;
  ros::NodeHandle nh("~");

  // SUBSCRIBER
  //ros::Subscriber sub = n.subscribe("shadow_effector_pose", 2, effectorPoseCallback);
  ros::Subscriber sub = n.subscribe("shadow_effector_pose_bodkin", 2, effectorPoseCallback);

  //ros::Subscriber sub_kinect = n.subscribe("/camera/depth/points", 2, pcl_kinect_Callback);
  ros::Subscriber sub_kinect = n.subscribe("/pc_server/PCLoud_visualizador", 2, pcl_kinect_Callback);
  ros::Subscriber sub_pose_vel = n.subscribe("/pose_vel_filt", 2, pose_vel_Callback);

  // Parametro rate
  int rate;
  nh.param<int>("rate", rate, 1000); //100
  //int rate = 100;
  ros::Rate loop_rate(rate);
  ROS_INFO("pc rate: %d Hz", rate);

  // Parametro frame_id
  std::string frame_id;
  //nh.param<std::string>("frame_id", frame_id, "/map");
  nh.param<std::string>("frame_id", frame_id, "/world");
  ROS_INFO("pc frame_id: %s", frame_id.c_str());

  ////////////////////////////////////////////////////////////////////////////////////
  ///////////   REDUCE CANTIDAD DE PUNTOS A TRAVES DE VOXELGRID  ///////////////////// 123456
  ////////////////////////////////////////////////////////////////////////////////////
  /*
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered2 (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PCDReader reader2;
  reader2.read ("/home/kuka/ei3001/grupo4/software/ros/phantom_kuka/table_scene.pcd", *cloud2); // Remember to download the file first!
  //reader2.read ("/home/david/ei3001/grupo4/software/ros/phantom_kuka/table_scene.pcd", *cloud2); // Remember to download the file first!
  pcl::VoxelGrid<pcl::PointXYZ> sor_XYZ;
  sor_XYZ.setInputCloud (cloud2);
  sor_XYZ.setLeafSize (0.02f, 0.02f, 0.02f); //is created with a leaf size of 2cm
  sor_XYZ.filter (*cloud_filtered2);
  // Transformacion entre PCL (version 2)
  pcl::PointCloud<pcl::PointXYZ> pc2=*cloud_filtered2; //**********************************************************************************
  pc2.header.frame_id = frame_id;


  // Se crea OCTREE
  float resolution = 512.0f;
  pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree (resolution);
  octree.setInputCloud(cloud_filtered2);
  octree.addPointsFromInputCloud();
  */

  // PUBLISHER
  //ros::Publisher pub = n.advertise<pcl::PointCloud<pcl::PointXYZ> >("point_cloud", 1);
  ros::Publisher marker_pub_R1 = n.advertise<visualization_msgs::Marker>("/phantom_control/visualization_marker_R1", 1);
  ros::Publisher marker_pub_R2 = n.advertise<visualization_msgs::Marker>("/phantom_control/visualization_marker_R2", 1);
  ros::Publisher marker_pub_R3 = n.advertise<visualization_msgs::Marker>("/phantom_control/visualization_marker_R3", 1);
  ros::Publisher marker_pub_norm = n.advertise<visualization_msgs::Marker>("/phantom_control/visualization_marker_norm", 1);
  ros::Publisher marker_pub_u_k = n.advertise<visualization_msgs::Marker>("/phantom_control/visualization_marker_u_k", 1);
  ros::Publisher marker_pub_force = n.advertise<visualization_msgs::Marker>("/phantom_control/visualization_marker_force", 1);
  //ros::Publisher u_k_vel_pub = n.advertise<geometry_msgs::Vector3Stamped>("/phantom_control/u_k_vel_topic", 1);

  // Publica Fuerzas a PHANTOM
  //ros::Publisher omni_forces_pub_PCL = n.advertise<geometry_msgs::WrenchStamped>("force_feedback",1);
  ros::Publisher omni_forces_pub_PCL = n.advertise<geometry_msgs::WrenchStamped>("force_in",1);
  ros::Publisher omni_forces_pub_PCL_dump = n.advertise<geometry_msgs::WrenchStamped>("force_in_dump",1);
  ros::Publisher HIP_position_pub = n.advertise<geometry_msgs::Vector3Stamped>("HIP_position_pub",1);
  ros::Publisher proxy_position_pub = n.advertise<geometry_msgs::Vector3Stamped>("proxy_position_pub",1);

  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::SPHERE;
  uint32_t shape_arrow = visualization_msgs::Marker::ARROW;

  // SE INICIALIZA POSICION DE PROXY
  proxy_position.x = HIP_position.x+0.2;
  proxy_position.y = HIP_position.y+0.2;
  proxy_position.z = HIP_position.z+0.2;
  proxy_position_mat.at<double>(0,0) = proxy_position.x;
  proxy_position_mat.at<double>(1,0) = proxy_position.y;
  proxy_position_mat.at<double>(2,0) = proxy_position.z;
  searchPoint.x = proxy_position.x;
  searchPoint.y = proxy_position.y;
  searchPoint.z = proxy_position.z;

  geometry_msgs::WrenchStamped forcePH;
  geometry_msgs::WrenchStamped forcePH_dump;
  forcePH.wrench.force.x = 0.0;
  forcePH.wrench.force.y = 0.0;
  forcePH.wrench.force.z = 0.0;
  forcePH_dump.wrench.force.x = 0.0;
  forcePH_dump.wrench.force.y = 0.0;
  forcePH_dump.wrench.force.z = 0.0;

  geometry_msgs::Vector3Stamped HIP_position_publicada;
  geometry_msgs::Vector3Stamped proxy_position_publicada;
  //geometry_msgs::Vector3Stamped u_k_vel_obj;

  u_k_norm_old=0.0;
  int seq = 0; // Se setea SEQ

  while (ros::ok()) {

    //while( ()||()||() ){
    //std::cerr << "searchPoint = "  << searchPoint << std::endl;
    octree.radiusSearch(searchPoint, radius_R1, pointIdxRadiusSearch_R1, pointRadiusSquaredDistance_R1);
    if(mostrar_info==1)
      std::cerr << "pointIdxRadiusSearch_R1.size() = "  << pointIdxRadiusSearch_R1.size() << std::endl;
    octree.radiusSearch(searchPoint, radius_R2, pointIdxRadiusSearch_R2, pointRadiusSquaredDistance_R2);
    if(mostrar_info==1)
      std::cerr << "pointIdxRadiusSearch_R2.size() = "  << pointIdxRadiusSearch_R2.size() << std::endl;
    octree.radiusSearch(searchPoint, radius_R3, pointIdxRadiusSearch_R3, pointRadiusSquaredDistance_R3);
    //std::cerr << "pointIdxRadiusSearch_R3.size() = "  << pointIdxRadiusSearch_R3.size() << std::endl;
    if(mostrar_info==1)
      std::cerr << "pointIdxRadiusSearch_R3.size() = "  << pointIdxRadiusSearch_R3.size() << std::endl;

    norm_vector = normal_vector(cloud_filtered2,pointIdxRadiusSearch_R3);
    calc_proxy_state(pointIdxRadiusSearch_R1,pointIdxRadiusSearch_R2,pointIdxRadiusSearch_R3);
    proxy_movement(proxy_state, norm_vector); // Se calcula la nueva posicion del PROXY
    //}


    calc_force(proxy_state); // Se calcula la fuerza al Phantom
    u_k = proxy_position_mat-HIP_position_mat;

    if (makers_on==1){
      /////////////////////////////////////////////////////////////
      // MARKER R1
      /////////////////////////////////////////////////////////////
      visualization_msgs::Marker markerR1;
      markerR1.header.frame_id = "/world";
      markerR1.header.stamp = ros::Time::now();
      markerR1.ns = "basic_shapes";
      markerR1.id = 0;
      markerR1.type = shape;
      markerR1.action = visualization_msgs::Marker::ADD;
      // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
      markerR1.pose.position.x = (double)(searchPoint.x);
      markerR1.pose.position.y = (double)(searchPoint.y);
      markerR1.pose.position.z = (double)(searchPoint.z);
      markerR1.pose.orientation.x = 0.0;
      markerR1.pose.orientation.y = 0.0;
      markerR1.pose.orientation.z = 0.0;
      markerR1.pose.orientation.w = 1.0;
      // Set the scale of the marker -- 1x1x1 here means 1m on a side
      markerR1.scale.x = 2*radius_R1;
      markerR1.scale.y = 2*radius_R1;
      markerR1.scale.z = 2*radius_R1;
      // Set the color -- be sure to set alpha to something non-zero!
      markerR1.color.r = 1.0f;
      markerR1.color.g = 0.0f;
      markerR1.color.b = 1.0f;
      markerR1.color.a = 0.3;
      markerR1.lifetime = ros::Duration();
      marker_pub_R1.publish(markerR1);
      /////////////////////////////////////////////////////////////
      // MARKER R2
      /////////////////////////////////////////////////////////////
      visualization_msgs::Marker markerR2;
      markerR2.header.frame_id = "/world";
      markerR2.header.stamp = ros::Time::now();
      markerR2.ns = "basic_shapes";
      markerR2.id = 0;
      markerR2.type = shape;
      markerR2.action = visualization_msgs::Marker::ADD;
      // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
      markerR2.pose.position.x = (double)(searchPoint.x);
      markerR2.pose.position.y = (double)(searchPoint.y);
      markerR2.pose.position.z = (double)(searchPoint.z);
      markerR2.pose.orientation.x = 0.0;
      markerR2.pose.orientation.y = 0.0;
      markerR2.pose.orientation.z = 0.0;
      markerR2.pose.orientation.w = 1.0;
      // Set the scale of the marker -- 1x1x1 here means 1m on a side
      markerR2.scale.x = 2*radius_R2;
      markerR2.scale.y = 2*radius_R2;
      markerR2.scale.z = 2*radius_R2;
      // Set the color -- be sure to set alpha to something non-zero!
      markerR2.color.r = 0.0f;
      markerR2.color.g = 1.0f;
      markerR2.color.b = 1.0f;
      markerR2.color.a = 0.3;
      markerR2.lifetime = ros::Duration();
      marker_pub_R2.publish(markerR2);
      /////////////////////////////////////////////////////////////
      // MARKER R3
      /////////////////////////////////////////////////////////////
      visualization_msgs::Marker markerR3;
      markerR3.header.frame_id = "/world";
      markerR3.header.stamp = ros::Time::now();
      markerR3.ns = "basic_shapes";
      markerR3.id = 0;
      markerR3.type = shape;
      markerR3.action = visualization_msgs::Marker::ADD;
      // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
      markerR3.pose.position.x = (double)(searchPoint.x);
      markerR3.pose.position.y = (double)(searchPoint.y);
      markerR3.pose.position.z = (double)(searchPoint.z);
      markerR3.pose.orientation.x = 0.0;
      markerR3.pose.orientation.y = 0.0;
      markerR3.pose.orientation.z = 0.0;
      markerR3.pose.orientation.w = 1.0;
      // Set the scale of the marker -- 1x1x1 here means 1m on a side
      markerR3.scale.x = 2*radius_R3;
      markerR3.scale.y = 2*radius_R3;
      markerR3.scale.z = 2*radius_R3;
      // Set the color -- be sure to set alpha to something non-zero!
      markerR3.color.r = 1.0f;
      markerR3.color.g = 1.0f;
      markerR3.color.b = 0.0f;
      markerR3.color.a = 0.3;
      markerR3.lifetime = ros::Duration();
      marker_pub_R3.publish(markerR3);

      /////////////////////////////////////////////////////////////
      // MARKER NORMAL
      /////////////////////////////////////////////////////////////
      visualization_msgs::Marker marker_norm;
      marker_norm.header.frame_id = "/world";
      marker_norm.header.stamp = ros::Time::now();
      //marker_norm.ns = "basic_shapes";
      //marker_norm.id = 0;
      marker_norm.ns = "arrow_normal";
      marker_norm.id = 0;
      marker_norm.type = shape_arrow;
      marker_norm.action = visualization_msgs::Marker::ADD;
      // Set the scale of the marker -- 1x1x1 here means 1m on a side
      marker_norm.scale.x = 0.05; //shaft diameter
      marker_norm.scale.y = 0.1;  //head diameter
      marker_norm.scale.z = 0.1;  //head length
      marker_norm.color.r = 1.0f;
      marker_norm.color.g = 1.0f;
      marker_norm.color.b = 1.0f;
      marker_norm.color.a = 1.0;
      // INICIO FLECHA
      marker_norm.points.resize(2);
      marker_norm.points[0].x = proxy_position.x;
      marker_norm.points[0].y = proxy_position.y;
      marker_norm.points[0].z = proxy_position.z;
      // FIN FLECHA
      marker_norm.points[1].x = proxy_position.x+(0.5*(norm_vector.at<double>(0,0)))+0.0001;
      marker_norm.points[1].y = proxy_position.y+(0.5*(norm_vector.at<double>(1,0)))+0.0001;
      marker_norm.points[1].z = proxy_position.z+(0.5*(norm_vector.at<double>(2,0)))+0.0001;
      marker_norm.lifetime = ros::Duration();
      marker_pub_norm.publish(marker_norm);

      /////////////////////////////////////////////////////////////
      // MARKER U_K
      /////////////////////////////////////////////////////////////
      visualization_msgs::Marker marker_u_k;
      marker_u_k.header.frame_id = "/world";
      marker_u_k.header.stamp = ros::Time::now();
      marker_u_k.ns = "arrow_normal";
      marker_u_k.id = 0;
      marker_u_k.type = visualization_msgs::Marker::ARROW;;
      marker_u_k.action = visualization_msgs::Marker::ADD;
      // Set the scale of the marker -- 1x1x1 here means 1m on a side
      marker_u_k.scale.x = 0.02; //shaft diameter
      marker_u_k.scale.y = 0.05;  //head diameter
      marker_u_k.scale.z = 0.05;  //head length
      marker_u_k.color.r = 0.9f;
      marker_u_k.color.g = 0.4f;
      marker_u_k.color.b = 0.0f;
      marker_u_k.color.a = 0.7;
      // INICIO FLECHA
      marker_u_k.points.resize(2);
      marker_u_k.points[0].x = proxy_position.x;
      marker_u_k.points[0].y = proxy_position.y;
      marker_u_k.points[0].z = proxy_position.z;
      // FIN FLECHA
      marker_u_k.points[1].x = proxy_position.x+(-u_k.at<double>(0,0))+0.0001;
      marker_u_k.points[1].y = proxy_position.y+(-u_k.at<double>(1,0))+0.0001;
      marker_u_k.points[1].z = proxy_position.z+(-u_k.at<double>(2,0))+0.0001;
      marker_u_k.lifetime = ros::Duration();
      marker_pub_u_k.publish(marker_u_k);

      /////////////////////////////////////////////////////////////
      // MARKER FORCE
      /////////////////////////////////////////////////////////////
      visualization_msgs::Marker marker_force;
      marker_force.header.frame_id = "/world";
      marker_force.header.stamp = ros::Time::now();
      marker_force.ns = "arrow_normal";
      marker_force.id = 0;
      marker_force.type = visualization_msgs::Marker::ARROW;;
      marker_force.action = visualization_msgs::Marker::ADD;
      // Set the scale of the marker -- 1x1x1 here means 1m on a side
      marker_force.scale.x = 0.03; //shaft diameter
      marker_force.scale.y = 0.06;  //head diameter
      marker_force.scale.z = 0.06;  //head length
      marker_force.color.r = 0.4f;
      marker_force.color.g = 0.0f;
      marker_force.color.b = 0.0f;
      marker_force.color.a = 1.0; //asd
      // INICIO FLECHA
      marker_force.points.resize(2);
      marker_force.points[0].x = HIP_position.x;
      marker_force.points[0].y = HIP_position.y;
      marker_force.points[0].z = HIP_position.z;
      // FIN FLECHA
      marker_force.points[1].x = HIP_position.x+(-Force_HIP.at<double>(0,0))+0.0001;
      marker_force.points[1].y = HIP_position.y+(-Force_HIP.at<double>(1,0))+0.0001;
      marker_force.points[1].z = HIP_position.z+(-Force_HIP.at<double>(2,0))+0.0001;
      marker_force.lifetime = ros::Duration();
      marker_pub_force.publish(marker_force);
    }

    //SE SETEA LA FUERZA AL PHANTOM

    /*
    forcePH.header.frame_id = "/link_0"; // OJO!!! REVISAR!!
    forcePH.header.seq++;
    forcePH.header.stamp=ros::Time::now();
    //TEST
    //forcePH.wrench.force.x = Force_HIP.at<double>(2,0); // OJO!! REVISAR SIGNOS!!!
    //forcePH.wrench.force.y = Force_HIP.at<double>(0,0); // OJO!! REVISAR SIGNOS!!!
    //forcePH.wrench.force.z = -Force_HIP.at<double>(1,0); // OJO!! REVISAR SIGNOS!!!
    forcePH.wrench.force.x = -Force_HIP.at<double>(0,0); // OJO!! REVISAR SIGNOS!!!
    forcePH.wrench.force.y = -Force_HIP.at<double>(1,0); // OJO!! REVISAR SIGNOS!!!
    forcePH.wrench.force.z = -Force_HIP.at<double>(2,0); // OJO!! REVISAR SIGNOS!!!
    */

    forcePH = force_outliers_filter(forcePH,Force_HIP);
    omni_forces_pub_PCL.publish(forcePH);
    forcePH_dump = force_outliers_filter_dump(forcePH_dump,Force_HIP_dump);
    omni_forces_pub_PCL_dump.publish(forcePH_dump);

    //SE PUBLICA LA HIP_position
    HIP_position_publicada.header.frame_id = "/world"; // OJO!!! REVISAR!!
    HIP_position_publicada.header.seq++;
    HIP_position_publicada.header.stamp = ros::Time::now();
    HIP_position_publicada.vector.x = HIP_position.x;
    HIP_position_publicada.vector.y = HIP_position.y;
    HIP_position_publicada.vector.z = HIP_position.z;
    HIP_position_pub.publish(HIP_position_publicada);
    //SE PUBLICA proxy_position
    proxy_position_publicada.header.frame_id = "/world"; // OJO!!! REVISAR!!
    proxy_position_publicada.header.seq++;
    proxy_position_publicada.header.stamp = ros::Time::now();
    proxy_position_publicada.vector.x = proxy_position.x;
    proxy_position_publicada.vector.y = proxy_position.y;
    proxy_position_publicada.vector.z = proxy_position.z;
    proxy_position_pub.publish(proxy_position_publicada);
    /*
    //SE PUBLICA u_k_vel
    u_k_vel_obj.header.frame_id = "/world";
    u_k_vel_obj.header.seq++;
    u_k_vel_obj.header.stamp=ros::Time::now();
    //u_k_vel_obj.vector.x=u_k_vel;
    u_k_vel_obj.vector.x=velocidad_HIP;
    //velocidad_HIP
    u_k_vel_pub.publish(u_k_vel_obj);
    */



    ros::spinOnce();   // Se ejecutan los CALLBACK
    loop_rate.sleep(); //delay variable para cumplir con rate pedido
  }
  return (0);
}

