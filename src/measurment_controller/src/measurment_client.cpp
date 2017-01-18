#include "measurmentManager.h"

/* @var g_measurmentManager
 * @brief 測定結果マネジャー
 */
measurmentManager *g_measurmentClient;

/* @var g_date_cnt
 * @brief 各列のデータ個数
 */
std::vector <int> g_date_cnt;

/* @struct csv_format_t
 * @brief csvフォーマット構造体定義
 */
typedef struct {
	double id;
	std::vector <std::string> date;
}  csv_format_t;

extern char *__progname;
//*****************************************************************************************
/*!
 *	@fn insertData
 *	@brief 測定データの挿入を行う
 *	@param[in] service    : サービス
 *  @param[in] csv_format_array : cvsフォーマット
 *  @param[in] csvPos : csv側の行の挿入位置
 *  @param[in] dataPos : data側の行の参照位置
 *  @param[in] dateCol : 日付データ挿入位置
 *	@return なし
 *	@details	
 *   ・応答データの設定処理を実施する。
 *	@author
 *	@par
 *	@throws
 *	@date
 *	 2016.11.08 新規作成 及川陽樹<br>
 */
//*****************************************************************************************
int insertData(measurment_controller::measurmentSrv *service,
						 csv_format_t *csv_format_array, int csvPos, int  dataPos, int dateCol ){
	int ret = -1;
	
	csv_format_array[csvPos].date[dateCol+0] = service->response.starttime[dataPos];
	csv_format_array[csvPos].date[dateCol+1] = service->response.endtime[dataPos];
	g_date_cnt[dateCol+0]++;
	g_date_cnt[dateCol+1]++;

	return ret;				
}

//*****************************************************************************************
/*!
 *	@fn searchInsertPosition
 *	@brief 測定データの挿入位置を検索する。
 *	@param[in] service : サービス
 *  @param[in] csv_format_array : csv構造体
 *  @param[in] size : サイズ
 *	@return なし
 *	@details
 *   ・取得したデータと一致するIDが存在するかをチェックする。
 *	@author
 *	@par
 *	@throws
 *	@date
 *	   2016.11.08 新規作成 及川陽樹<br>
 */
//*****************************************************************************************
int searchInsertPosition(measurment_controller::measurmentSrv *service,
											csv_format_t *csv_format_array, int size, int dateCol ){
	int ret = -1;
	//! 挿入するデータ位置を探索する。	
	for( int i = 0; i  < (int)service->response.cnt; i++ ){
			for( int j = 0; j < size; j++ ){
					if( service->response.mnid[i] == csv_format_array[j].id ){
						ret = 0;
						insertData( service, &csv_format_array[0], j, i,dateCol );
					}
			}
	}

	return ret;				
}
//*****************************************************************************************
/*!
 *	@fn createFile
 *	@brief 測定データの挿入を行う
 *	@param[in] void    : なし
 *	@return なし
 *	@details
 *   ・性能測定データのファイル出力を実施する。
 *	@author
 *	@par
 *	@throws
 *	@date
 *	 2016.11.08 新規作成 及川陽樹<br>
 */
//*****************************************************************************************
int createFile( csv_format_t *csv_format, int size ){

	unsigned char folder_name[256];
	unsigned char fname[256];
	unsigned char tstr[256];
	unsigned char str[256];
	std::ofstream wstream;
	struct tm tm;
	time_t t; 
	int ret = 0;
	  
	memset( folder_name, 0x00,sizeof(folder_name) );
	memset( fname, 0x00,sizeof(fname) );
		
	sprintf( (char*)folder_name, "%s",g_measurmentClient->path.c_str());
	mkdir( (char*)folder_name,  0777);

	t = time(NULL);
	localtime_r(&t,&tm);

	//! ファイル名称を作成する。
	strftime( (char*)tstr,sizeof(str),"%Y%m%d-%H%M%S",&tm);
	sprintf( (char*)fname,"%s/%s_ret.csv",(char*)folder_name,(char*)tstr );
	
	//!  ファイルオープンを実施する。			
	wstream.open( (const char*)fname, std::ios::out | std::ios::app );

	//!  行の出力を実施する。
	int i = 0, j = 0;
	for( i = 0; i < size; i++ ){
			memset(str,0x00,sizeof(str));
			sprintf((char*)str,"%lf",csv_format[i].id);
			wstream << str;
			for( j = 0; j < (int)csv_format[i].date.size(); j++ ){
				if( g_date_cnt[j] > 0 ){
					wstream << "," << csv_format[i].date[j];
				}
			}
			wstream << std::endl;
	}			

	wstream.close();

	return ret;				
}
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
	unsigned char service_name[256];
	measurment_controller::measurmentSrv service;

	std::vector<int> thread_array;
	std::vector<int> type_array;
	std::vector <std::string> nodes;
	std::vector <std::string> srvs;
	std::vector <csv_format_t> csv_format_array;
	std::vector<ros::ServiceClient> clients;
	std::string path;
		
	ros::init(argc, argv, "measurment_client_node");		
	
	ros::NodeHandle n;
	//! 測定クラスの初期化処理を実施する。
	g_measurmentClient = new measurmentManager();
	
	//! ファイルパスを取得する。
	ros::param::get("/measurment_client_node/path",  g_measurmentClient->path);		
	ros::param::get("/measurment_client_node/monitor/thread",thread_array);
	ros::param::get("/measurment_client_node/monitor/type",type_array);
	ros::param::get("/measurment_client_node/monitor/nodes",nodes);
	ros::param::get("/measurment_client_node/monitor/srvs",srvs);
	
	//! サービス名称を作成する。
	for( int node = 0; node < (int)nodes.size(); node++){
		sprintf(  (char*)service_name, "/%s/%s", nodes[node].c_str(), srvs[node].c_str() );
		clients.push_back(n.serviceClient<measurment_controller::measurmentSrv>((char*)service_name));
	}
							
	//! 測定結果を表示させる。
	service.request.thread_id = 0;
	service.request.type = 0;
	g_date_cnt.resize(nodes.size()*thread_array.size()*type_array.size()*2);

	if( clients[0].call( service ) == false ){
		ROS_ERROR("service call error");
	}else{
			if( (service.response.begin == 0)&&(service.response.end == 0) ){
				return 0;
			}
			//! IDの列を作成する。
			for( int i = 0; i  < (int)service.response.cnt; i++ ){
				csv_format_t csv_format;
				csv_format_array.push_back(csv_format);
				csv_format_array[i].id = service.response.mnid[i];
				csv_format_array[i].date.resize(nodes.size()*thread_array.size()*type_array.size()*2);
			}
	}

	//! 各ノードのデータを取得する。
	int dateCol = 0;
	for( int node = 0; node < (int)nodes.size(); node++ ){
		for( int thread = 0; thread < (int)thread_array.size(); thread++ ){
			for( int type = 0; type < (int)type_array.size(); type++ ){
				//! イメージの待受処理を実施する。
				service.request.thread_id = thread_array[thread];
				service.request.type = type_array[type];
				if( clients[node].call( service ) == false ){
					ROS_ERROR("service call error %d %d %d",node,thread,type);
				}else{
					searchInsertPosition( &service, &csv_format_array[0], csv_format_array.size(), dateCol );
				}
				dateCol+=2;
			}
		}
	}			

	createFile( &csv_format_array[0], csv_format_array.size() );					

	return 0;
}

