#include "measurmentManager.h"

/* @var g_measurment_table_cnt
 * @brief 測定のカウンタ
 */ 
int g_measurment_table_cnt = 0;

//*****************************************************************************************
/*!
 *	@fn measurmentManager
 *	@brief 測定マネージャーのコンストラクタ
 *	@param[in] void :なし
 *	@details
 *    測定マネージャーのコンストラクタ。<br>
 *	@author
 *	@par
 *	@throws
 *	@date
 *	 2016.11.06 新規作成 及川陽樹<br>
 */
//*****************************************************************************************
measurmentManager::measurmentManager(){
	return;
}  
//*****************************************************************************************
/*!
 *	@fn measurmentManager
 *	@brief 測定マネージャーのデストラクタ
 *	@param[in] void :なし
 *	@details
 *    測定マネージャーのデストラクタ。<br>
 *	@author
 *	@par
 *	@throws
 *	@date
 *	 2016.11.06 新規作成 及川陽樹<br>
 */
//*****************************************************************************************
measurmentManager::~measurmentManager(){
	return;
}  
//*****************************************************************************************
/*!
 *	@fn service
 *	@brief 測定情報を提供するサービス
 *	@param[in] type : 処理
 *	@return true	: 正常
 *          false	: 異常
 *	@details
 * 　  該当処理の性能情報を取得するサービス<br>
 *	@author
 *	@par
 *	@throws
 *	@date
 *	 2016.11.06 新規作成 及川陽樹<br>
 */
//*****************************************************************************************
bool measurmentManager::service(  
	 measurment_controller::measurmentSrv::Request &req, 
	 measurment_controller::measurmentSrv::Response &res ){
	
	int count = 0;
	
	//! 初期状態の場合は、falseを応答する。
	if( this->measurmentTable[req.thread_id][req.type].status == D_MEASURMENT_INIT ){
		return false;
	}
	//!  既に一周している場合は、全配列のデータ数を設定する。
	if( this->measurmentTable[req.thread_id][req.type].cycle > 0 ){
		count = D_MEASURMENT_RAW_MAX;
		//! 読み取り開始位置と、終了位置を指定する。
		res.begin = this->measurmentTable[req.thread_id][req.type].cnt;
		if( this->measurmentTable[req.thread_id][req.type].cnt == 0 ){
				res.end = (D_MEASURMENT_RAW_MAX - 1);
		}else{
				res.end = this->measurmentTable[req.thread_id][req.type].cnt - 1;
		}
	} else {
		count = this->measurmentTable[req.thread_id][req.type].cnt; 
		res.begin = 0;
		if( this->measurmentTable[req.thread_id][req.type].cnt == 0 ){
			res.end = 0;
		}else{
			res.end = this->measurmentTable[req.thread_id][req.type].cnt - 1;
		}
	}
	
	//! プロセス名称を設定する。
	std::string str( (const char*)this->measurmentTable[req.thread_id][req.type].process_name );
	
	//! 生値の設定を実施する。 
	for( int i = 0; i < count; i++ ){
		std::string start_date_str( (const char*)this->measurmentTable[req.thread_id][req.type].StartDate[i] );
		std::string end_date_str( (const char*)this->measurmentTable[req.thread_id][req.type].EndDate[i] );
		res.raw.push_back(this->measurmentTable[req.thread_id][req.type].raw[i]);	
	    res.mnid.push_back(this->measurmentTable[req.thread_id][req.type].mnid[i]);
		res.starttime.push_back(start_date_str);	
		res.endtime.push_back(end_date_str);	
	}

	//!  最小、最大の値を応答する。
	res.name = str;
	res.min = this->measurmentTable[req.thread_id][req.type].min;
	res.max = this->measurmentTable[req.thread_id][req.type].max;
	res.cnt   = count;	 
	return true;
}
//*****************************************************************************************
/*!
 *	@fn processThroughputEntry
 *	@brief 該当処理の性能ポイントをエントリするメソッド
 *	@param[in]  thread_id : スレッドID
 *  @param[in] type : 処理
 *	@return true	: 正常
 *　　          false	: 異常
 *	@details
 *     該当処理の性能ポイントの登録処理を実施する。<br>
 *	@author
 *	@par
 *	@throws
 *	@date
 *	 2016.11.06 新規作成 及川陽樹<br>
 */	
//*****************************************************************************************
bool measurmentManager::processThroughputEntry(int thread_id, int type,std::string name ){
	
	//! スレッドIDが最大値を超えている場合は、エラーを応答する。
	if( thread_id > D_THREAD_ID_MAX ){
		ROS_ERROR("argument thread_id mistake.");
		return false;
	}
	//! 種別が最大を超えている場合は、エラーを応答する。
	if( type > D_MEASURMENT_MAX ){
		ROS_ERROR("argument type mistake.");
		return false;
	}
	//! 測定のカウンタを上げる。
	if( g_measurment_table_cnt >= (D_MEASURMENT_MAX*D_THREAD_ID_MAX)){
		ROS_ERROR("measurment over.");
		return false;
	}	
	//! 既に登録済みの場合は、falseを応答する。
	if( this->measurmentTable[thread_id][type].status !=  D_MEASURMENT_INIT ){
		ROS_ERROR("measurment is already entry. %d, %d", thread_id,type);
		return false;
	}
	//! 測定情報を初期化する。
	memset( &this->measurmentTable[thread_id][type],0x00,sizeof(this->measurmentTable[thread_id][type]));
	strcpy( (char*)this->measurmentTable[thread_id][type].process_name,(const char*)name.c_str() );	
	this->measurmentTable[thread_id][type].min = 9999.0;
	
	this->measurmentTable[thread_id][type].status = D_MEASURMENT_WAITING;

	g_measurment_table_cnt++;

	return true;
}
//*****************************************************************************************
/*!
 *	@fn processThroughputStart
 *	@brief 該当処理の性能ポイントの測定を開始するメソッド
 *	@param[in]  thread_id : スレッドID
 *  @param[in] type : 処理
 *	@return true	: 正常
 *                 false	: 異常
 *	@details
 *     該当処理のスループット測定を開始する。<br>
 *	@author
 *	@par
 *	@throws
 *	@date
 *	 2016.11.06 新規作成 及川陽樹<br>
 */
//*****************************************************************************************
bool measurmentManager::processThroughputStart(int thread_id, int type){

	struct tm localTime;
	
	//! スレッドIDが最大値を超えている場合は、エラーを応答する。
	if( thread_id > D_THREAD_ID_MAX ){;
		ROS_ERROR("argument thread_id mistake.");
		return false;
	}
	//! 種別が最大を超えている場合は、エラーを応答する。
	if( type > D_MEASURMENT_MAX ){
		ROS_ERROR("argument type mistake.");
		return false;
	}

	if(this->measurmentTable[thread_id][type].status != D_MEASURMENT_WAITING){
		ROS_ERROR("measurmentStatus is not waiting %d %d %d",
				thread_id,type,this->measurmentTable[thread_id][type].status);
		return false;
	}
	
	if( this->measurmentTable[thread_id][type].cnt >= D_MEASURMENT_RAW_MAX ){
		this->measurmentTable[thread_id][type].cnt = 0;
		this->measurmentTable[thread_id][type].cycle++;
	}

	//! 開始時間を記憶する。
	this->measurmentTable[thread_id][type].start = std::chrono::system_clock::now() ;
	const std::chrono::duration<double> tse = this->measurmentTable[thread_id][type].start.time_since_epoch();
	std::chrono::seconds::rep milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(tse).count() % 1000;		
	time_t now = std::chrono::system_clock::to_time_t(this->measurmentTable[thread_id][type].start);	
	localtime_r(&now, &localTime);
	
	sprintf( (char*)this->measurmentTable[thread_id][type].StartDate[this->measurmentTable[thread_id][type].cnt ],
				 "%04d-%02d-%02d %02d:%02d:%02d.%03d",
				 (1900 + localTime.tm_year),
				 (localTime.tm_mon + 1) ,
				 localTime.tm_mday,localTime.tm_hour,localTime.tm_min,localTime.tm_sec,
				 (int)milliseconds );
	this->measurmentTable[thread_id][type].status = D_MEASURMENT_START;
	this->measurmentTable[thread_id][type].mnid[this->measurmentTable[thread_id][type].cnt] = 0;

	return true;
}
//*****************************************************************************************
/*!
 *	@fn processThroughputEnd
 *	@brief 該当処理のスループットを測定するメソッド[終了]
 *	@param[in]  thread_id : スレッドID
 *  @param[in] type : 処理
 *	@return true	 : 正常
 *                 false : 異常
 *	@details
 *     該当処理のスループット測定を終了する。<br>
 *	@author
 *	@par
 *	@throws
 *	@date
 *　	 2016.11.06 新規作成 及川陽樹<br>
 */
//*****************************************************************************************
bool measurmentManager::processThroughputEnd(int thread_id, int type){

	struct tm localTime;

	//! スレッドIDが登録可能な最大値を超えている場合は、エラーを応答する。
	if( thread_id > D_THREAD_ID_MAX ){
		ROS_ERROR("argument thread_id mistake.");
		return false;
	}
	//! 種別が登録可能な最大値を超えている場合は、エラーを応答する。
	if( type > D_MEASURMENT_MAX ){
		ROS_ERROR("argument type mistake.");
		return false;
	}

	//! 測定中以外の場合は、falseを応答する。
	if(this->measurmentTable[thread_id][type].status != D_MEASURMENT_START){
		ROS_ERROR("measurmentStatus is not start.%d %d",thread_id,type);
		return false;
	}
	
	//! 終了時間を記憶する。
	this->measurmentTable[thread_id][type].end = std::chrono::system_clock::now();

	//! 処理時間を記憶する。
	this->measurmentTable[thread_id][type].raw[this->measurmentTable[thread_id][type].cnt]= 
		(std::chrono::duration_cast<std::chrono::milliseconds>(
			this->measurmentTable[thread_id][type].end - this->measurmentTable[thread_id][type].start ).count())/1000.0;

	//! 最小時間を記憶する。
	if( this->measurmentTable[thread_id][type].min > 
	    this->measurmentTable[thread_id][type].raw[this->measurmentTable[thread_id][type].cnt] ){
		this->measurmentTable[thread_id][type].min = 
			this->measurmentTable[thread_id][type].raw[this->measurmentTable[thread_id][type].cnt];
	}
	//! 最大値を記憶する。
	if( this->measurmentTable[thread_id][type].max < 
	    this->measurmentTable[thread_id][type].raw[this->measurmentTable[thread_id][type].cnt] ){
		this->measurmentTable[thread_id][type].max = 
			this->measurmentTable[thread_id][type].raw[this->measurmentTable[thread_id][type].cnt];
	}

	time_t now = std::chrono::system_clock::to_time_t(this->measurmentTable[thread_id][type].end);	
	localtime_r(&now, &localTime);
	const std::chrono::duration<double> tse = this->measurmentTable[thread_id][type].end.time_since_epoch();
	std::chrono::seconds::rep milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(tse).count() % 1000;		

	sprintf( (char*)this->measurmentTable[thread_id][type].EndDate[this->measurmentTable[thread_id][type].cnt ],
				 "%04d-%02d-%02d %02d:%02d:%02d.%03d",
				 (1900 + localTime.tm_year),
				 (localTime.tm_mon + 1) ,
				 localTime.tm_mday,localTime.tm_hour,localTime.tm_min,localTime.tm_sec,
				 (int)milliseconds );
	
	//! ステータスを待ちにする。
	this->measurmentTable[thread_id][type].status = D_MEASURMENT_WAITING;

	//! 次レコードに設定する。
	this->measurmentTable[thread_id][type].cnt++;

	return true;
}
//*****************************************************************************************
/*!
 *	@fn processThroughputThreadStart
 *	@brief キーIDを指定して測定を開始するメソッド
 *	@param[in]  thread_id : スレッドID
 *  @param[in] type : 処理
 *  @param[in] mnid :  mnid 
 *	@return true	: 正常	
 *                 false	: 異常
 *	@details
 *     キーIDを指定して該当処理の計測を開始する。<br>
 *	@author
 *	@par
 *	@throws
 *	@date
 *	 2016.11.06 新規作成 及川陽樹<br>
 */
//*****************************************************************************************
bool measurmentManager::processThroughputThreadStart(int thread_id, int type,double mnid ){

	struct tm localTime;
	
	//! スレッドIDが登録可能な最大値を超えている場合は、エラーを応答する。
	if( thread_id > D_THREAD_ID_MAX ){;
		ROS_ERROR("argument thread_id mistake.");
		return false;
	}
	//! 種別が登録可能な最大値を超えている場合は、エラーを応答する。
	if( type > D_MEASURMENT_MAX ){
		ROS_ERROR("argument type mistake.");
		return false;
	}

	if(this->measurmentTable[thread_id][type].status != D_MEASURMENT_WAITING){
		ROS_ERROR("measurmentStatus is not waiting %d %d %d",
				thread_id,type,this->measurmentTable[thread_id][type].status);
		return false;
	}
	
	if( this->measurmentTable[thread_id][type].cnt >= D_MEASURMENT_RAW_MAX ){
		this->measurmentTable[thread_id][type].cnt = 0;
		this->measurmentTable[thread_id][type].cycle++;
	}

	//! 開始時間を記憶する。
	this->measurmentTable[thread_id][type].start = std::chrono::system_clock::now() ;
	const std::chrono::duration<double> tse = this->measurmentTable[thread_id][type].start.time_since_epoch();
	std::chrono::seconds::rep milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(tse).count() % 1000;		
	time_t now = std::chrono::system_clock::to_time_t(this->measurmentTable[thread_id][type].start);	
	localtime_r(&now, &localTime);
	
	sprintf( (char*)this->measurmentTable[thread_id][type].StartDate[this->measurmentTable[thread_id][type].cnt ],
				 "%04d-%02d-%02d %02d:%02d:%02d.%03d",
				 (1900 + localTime.tm_year),
				 (localTime.tm_mon + 1) ,
				 localTime.tm_mday,localTime.tm_hour,localTime.tm_min,localTime.tm_sec,
				 (int)milliseconds );
				 
	this->measurmentTable[thread_id][type].status = D_MEASURMENT_START;
	this->measurmentTable[thread_id][type].mnid[this->measurmentTable[thread_id][type].cnt] = mnid;

	return true;
}
//*****************************************************************************************
/*!
 *	@fn processThroughputThreadEnd
 *	@brief キーIDを指定して測定を終了するメソッド
 *	@param[in]  thread_id : スレッドID
 *  @param[in] type : 処理
 *	@return true	 : 正常
 *                 false : 異常
 *	@details
 *     該当処理のスループット測定を終了する。<br>
 *	@author
 *	@par
 *	@throws
 *	@date
 *	 2016.11.06 新規作成 及川陽樹<br>
 */
//*****************************************************************************************
bool measurmentManager::processThroughputThreadEnd(int thread_id, int type,double mnid){

	struct tm localTime;

	//! スレッドIDが登録可能な最大値を超えている場合は、エラーを応答する。
	if( thread_id > D_THREAD_ID_MAX ){
		ROS_ERROR("argument thread_id mistake.");
		return false;
	}
	//! 種別が登録可能な最大値を超えている場合は、エラーを応答する。
	if( type > D_MEASURMENT_MAX ){
		ROS_ERROR("argument type mistake.");
		return false;
	}

	//! 測定中以外の場合は、falseを応答する。
	if(this->measurmentTable[thread_id][type].status != D_MEASURMENT_START){
		ROS_ERROR("measurmentStatus is not start.%d %d",thread_id,type);
		return false;
	}
	
	//! 終了時間を記憶する。
	this->measurmentTable[thread_id][type].end = std::chrono::system_clock::now();

	//! 処理時間を記憶する。
	this->measurmentTable[thread_id][type].raw[this->measurmentTable[thread_id][type].cnt]= 
		(std::chrono::duration_cast<std::chrono::milliseconds>(
			this->measurmentTable[thread_id][type].end - this->measurmentTable[thread_id][type].start ).count())/1000.0;

	//! 最小時間を記憶する。
	if( this->measurmentTable[thread_id][type].min > 
	    this->measurmentTable[thread_id][type].raw[this->measurmentTable[thread_id][type].cnt] ){
		this->measurmentTable[thread_id][type].min = 
			this->measurmentTable[thread_id][type].raw[this->measurmentTable[thread_id][type].cnt];
	}
	//! 最大値を記憶する。
	if( this->measurmentTable[thread_id][type].max < 
	    this->measurmentTable[thread_id][type].raw[this->measurmentTable[thread_id][type].cnt] ){
		this->measurmentTable[thread_id][type].max = 
			this->measurmentTable[thread_id][type].raw[this->measurmentTable[thread_id][type].cnt];
	}

	time_t now = std::chrono::system_clock::to_time_t(this->measurmentTable[thread_id][type].end);	
	localtime_r(&now, &localTime);
	const std::chrono::duration<double> tse = this->measurmentTable[thread_id][type].end.time_since_epoch();
	std::chrono::seconds::rep milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(tse).count() % 1000;		

	sprintf( (char*)this->measurmentTable[thread_id][type].EndDate[this->measurmentTable[thread_id][type].cnt ],
				 "%04d-%02d-%02d %02d:%02d:%02d.%03d",
				 (1900 + localTime.tm_year),
				 (localTime.tm_mon + 1) ,
				 localTime.tm_mday,localTime.tm_hour,localTime.tm_min,localTime.tm_sec,
				 (int)milliseconds );
	
	//! ステータスを待ちにする。
	this->measurmentTable[thread_id][type].status = D_MEASURMENT_WAITING;

	//! 生値のレコード数が最大値に達した場合は、一度リセットする。
	this->measurmentTable[thread_id][type].mnid[this->measurmentTable[thread_id][type].cnt] = mnid;
	this->measurmentTable[thread_id][type].cnt++;
	return true;
}
//*****************************************************************************************
/*!
 *	@fn processThroughputDelete
 *	@brief 指定された測定ポイントを削除するメソッド
 *	@param[in] thread_id : スレッドID
 *	@param[in] type : 種別
 *	@return true	: 正常
 *        　     false	: 異常
 *	@details
 *   該当処理の測定。<br>
 *	@author
 *	@par
 *	@throws
 *	@date
 *	 2016.11.06 新規作成 及川陽樹<br>
 */
//*****************************************************************************************
bool measurmentManager::processThroughputDelete(int  thread_id, int type){
	
	//! スレッドIDが最大値を超えている場合は、エラーを応答する。
	if( thread_id > D_THREAD_ID_MAX ){
		ROS_ERROR("argument thread_id mistake.");
		return false;
	}
	//! 種別が最大を超えている場合は、エラーを応答する。
	if( type > D_MEASURMENT_MAX ){
		ROS_ERROR("argument type mistake.");
		return false;
	}

	//! 測定のカウンタを上げる。
	if( g_measurment_table_cnt <= D_MEASURMENT_MIN ){
		ROS_ERROR("measurment not entry.");
		return false;
	}
	
	//! 測定情報を初期化する。
	memset( &this->measurmentTable[thread_id][type], 0x00, sizeof(measurment_t) );	
	this->measurmentTable[thread_id][type].min = 9999.0;		
	this->measurmentTable[thread_id][type].status = D_MEASURMENT_INIT;

	g_measurment_table_cnt--;

	return true;
}

