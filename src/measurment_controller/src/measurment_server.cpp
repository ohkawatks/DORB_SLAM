#include "measurmentManager.h"

/* @var g_measurmentManager
 * @brief 測定結果マネジャー
 */
measurmentManager *	g_measurmentServer;

//*****************************************************************************************
/*!
 *	@fn main
 *	@brief 測定の監視処理を実施するメイン関数
 *	@param[in] void    : なし
 *	@return なし
 *	@details
 *   ・ROSノードを生成し、測定処理を実施するメイン関数。
 *	@author
 *	@par
 *	@throws
 *	@date
 *	 2016.11.08 新規作成 及川陽樹<br>
 */
//*****************************************************************************************
int main(int argc, char **argv){
	measurment_controller::measurmentSrv service;
	
	ros::init(argc, argv, "measurment_server_node");		
	ros::NodeHandle n;
	std::string service_name;
	
	//! 測定クラスの初期化処理を実施する。	
	g_measurmentServer = new measurmentManager();
	
	ros::param::get("/measurment_server_node/service_name",service_name);
	ros::ServiceServer server = n.advertiseService(service_name, 
																					&measurmentManager::service, g_measurmentServer);
	
	//! 10Hzでループ処理を実施する。
	ros::Rate loop_rate(10);
	//! イメージの待受処理を実施する。
	service.request.thread_id = 1;
	service.request.type = 0;

	g_measurmentServer->processThroughputEntry(0,0,"ORB_SLAM2::LocalMapping::Run");
	while ( ros::ok() ){
		g_measurmentServer->processThroughputStart(0,0);
		ros::spinOnce();
		loop_rate.sleep();
		g_measurmentServer->processThroughputEnd(0,0);
	 }   
	g_measurmentServer->processThroughputDelete(0,0);

	return 0;
}

