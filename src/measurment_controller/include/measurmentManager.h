//for ros API
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/select.h>
#include <sys/time.h>
#include <unistd.h>
#include <iostream>
#include <fstream>
#include <time.h>
#include <errno.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <chrono>
#include "ros/ros.h"
#include "measurment_controller/measurmentSrv.h"
 
#ifndef D_MEASURMENT_MANAGER_H
#define D_MEASURMENT_MANAGER_H

/* 
 **********************************************************
 * 定数定義
 **********************************************************
 */
/* @def D_MEASURMENT_MIN
 * @brief 測定結果を格納する配列の最大値
 */
#define D_MEASURMENT_MIN	(0)
/* @def D_MEASURMENT_MAX
 * @brief 測定結果を格納する配列の最大値
 */
#define D_MEASURMENT_MAX	(5)

/* @def D_MEASURMENT_RAW_MAX
 * @brief 測定結果の生値を定義する
 */
#define D_MEASURMENT_RAW_MAX (6024)

 /* @def D_THREAD_ID_MAX 
 * @brief  スレッドIDの最大値
 */
#define D_THREAD_ID_MAX (5)

/* 
 **********************************************************
 * 構造体定義
 **********************************************************
 */
/* @struce measurment_t
 * @brief 測定結果を格納する構造体
 */
typedef struct {
	int status;
	int cnt;
	int cycle;
	int mnid_count;
	double mnid[D_MEASURMENT_RAW_MAX];
	std::chrono::system_clock::time_point start;
	std::chrono::system_clock::time_point end;
	double raw[D_MEASURMENT_RAW_MAX];
	double min;
	double max;
	unsigned char process_name[D_MEASURMENT_RAW_MAX];
	unsigned char StartDate[D_MEASURMENT_RAW_MAX][64];
	unsigned char EndDate[D_MEASURMENT_RAW_MAX][64];
} measurment_t;	

/* @def D_MEASURMENT_INIT
 * @brief 初期状態
 */
#define	D_MEASURMENT_INIT				(0)
/* @def D_MEASURMENT_WAITING
 * @brief 待ち状態
 */
#define D_MEASURMENT_WAITING	(1)
/* @def D_MEASURMENT_START
 * @brief 開始状態
 */
#define D_MEASURMENT_START			(2)

/* 
 **********************************************************
 *  @class measurmentManager
 *  @brief 測定用のクラス
 **********************************************************
 */
class measurmentManager {
public:
/***********************************************************
 * プロパティ定義
 **********************************************************
 */
	/* @var server
	 * @brief サーバ変数
	 */ 
	ros::ServiceServer server;
	/* @var client
	 * @brief クライアント変数
	 */ 	
	ros::ServiceClient client;
	/* @var measurmentTable
	 * @brief 測定用のテーブル
	 */ 
 	measurment_t measurmentTable[D_THREAD_ID_MAX][D_MEASURMENT_MAX];
	/* @var path
	 * @brief  出力用のファイルパス
	 */ 
	std::string path;
/* 
 **********************************************************
 * メソッド定義
 **********************************************************
 */ 
 
	/*
   	 * @sa measurmentManager
 	 */
   measurmentManager();
	/*
   	 * @sa ~measurmentManager
 	 */
   ~measurmentManager();
	/*
   	 * @sa service
 	 */
	bool service(  measurment_controller::measurmentSrv::Request &req, 
						    measurment_controller::measurmentSrv::Response &res );
	/*
   	 * @sa processThroughputEntry
 	 */
	bool processThroughputEntry(int thread_id, int type,std::string name);
	/*
   	 * @sa processThroughputStart
 	 */
	bool processThroughputStart(int thread_id, int type);
	/*
   	 * @sa processThroughputThreadStart
 	 */
	bool processThroughputThreadStart(int thread_id, int type,double mnid );
	/*
   	 * @sa processThroughputThreadEnd
 	 */	
	bool processThroughputThreadEnd(int thread_id, int type,double mnid);
	/*
   	 * @sa processThroughputEnd
 	 */
	bool processThroughputEnd(int thread_id, int type);
	/*
   	 * @sa processThroughputDelete
 	 */
	bool processThroughputDelete(int thread_id, int type);
};
#endif

