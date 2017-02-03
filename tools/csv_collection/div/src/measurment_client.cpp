#include <iostream>  // for debug writing
#include <string>  	  // useful for reading and writing

#include <fstream>    // ifstream, ofstream
#include <sstream>   // istringstream

#include <string.h>
#include <stdlib.h>

extern char *__progname;

unsigned long g_timeStrArray[6192][32][256];
unsigned long g_timeArray[6192][32];

double  g_procTimeArray[6192][32];
double  g_all_procTime_sum[32];
double  g_all_procTime_avg[32];
double  g_all_procTime_min[32];
double  g_all_procTime_max[32];
unsigned long  g_all_procTime_cnt[32];
	
unsigned long g_all_proctime = 0;
unsigned long g_all_proctime_max = 0;
unsigned long g_all_proctime_min = 0xFFFFFFFF;

unsigned long g_image_raw_lost = 0;
unsigned long g_orb_descriptor_lost = 0;
unsigned long g_tracker_state_lost = 0;
unsigned long g_localmapping_lost = 0;
unsigned long g_loopclosing_lost = 0;

unsigned long g_keyframe_proc_time = 0; 
unsigned long g_prev_keyframe_time = 0;  
unsigned long g_keyframe_proc_time_sum = 0;  
unsigned long g_keyframe_proc_time_avg  = 0; 
unsigned long g_keyframe_proc_time_max = 0;  
unsigned long g_keyframe_proc_time_min = 9999;  
unsigned long g_keyframe_proc_time_cnt = 0;  


//*****************************************************************************************
/*!
 *	@fn str2time
 *	@brief 時間文字列をミリ秒に変換するメソッド
 *	@param[in]  buf : 時間文字列
 *	@return true	 : 正常
 *                 false : 異常
 *	@details
 *     時間文字列を時間に変換する。<br>
 *	@author
 *	@par
 *	@throws
 *	@date
 *  　2017.01.21 新規作成 及川陽樹<br>
 */
//*****************************************************************************************
long str2time( unsigned char *buf ){

	int hour = 0, min = 0, sec = 0,  milisecond = 0;
	int year = 0, month = 0, day = 0;
	char split[] = " "; 
	char *tok;
	unsigned char wk[256];
	char date_str[256];
	char time_str[256];
	unsigned long time = 0;

	memset( date_str, 0x00, sizeof(date_str)  );
	memset( time_str, 0x00, sizeof(time_str) );
	memset( wk,          0x00, sizeof(wk) );

	//! 処理するべき文字列が存在しない場合は、エラーを応答する。
	if( strlen((const char*)buf) == 0 ){	
		return -1;
	}
	memcpy(wk,buf,strlen((char*)buf));
	
	//! 時間文字列をミリ秒に変換する。
	tok					= strtok((char*)wk,(const char*)split); 
	memcpy(date_str,tok,sizeof(date_str));
	split[0]			= '.';
	tok					= strtok(NULL,split); 
	memcpy(time_str,tok,sizeof(time_str));

	tok					= strtok(NULL,split); 
	milisecond		= atoi(tok);


	split[0]			= '-';
	tok					= strtok((char*)date_str,(const char*)split); 
	year              	= atoi(tok);
 	tok		            = strtok(NULL,split);
	month           	= atoi(tok);
	tok   	        	= strtok(NULL,split);
	day           	    = atoi(tok);

	split[0]	= ':';
	tok		= strtok(time_str,split);
	hour	= atoi(tok);
 	tok		= strtok(NULL,split);
	min		= atoi(tok);
 	tok		= strtok(NULL,split);
	sec		= atoi(tok);

	time = (day*(24*3600000)) + (hour*3600000) + (min * 60000) + (sec*1000) + milisecond; 

	return time;

}
//*****************************************************************************************
/*!
 *	@fn splitcsv
 *	@brief 時間文字列をミリ秒に変換するメソッド
 *	@param[in]  buf : 時間文字列
 *	@return true	 : 正常
 *                 false : 異常
 *	@details
 *     時間文字列を時間に変換する。<br>
 *	@author
 *	@par
 *	@throws
 *	@date
 *  　2017.01.21 新規作成 及川陽樹<br>
 */
//*****************************************************************************************
unsigned long splitcsv( unsigned char *buf, unsigned char* csv_data, unsigned long *data, int offset,unsigned char* rowInfoStr ){

	unsigned char split[2];
	unsigned char wk[2048];
	char *tok;
	int i = 0, j = 0;
	int cols = 0;
	int len = 0;
	int start = 0;
	int pos = 0;
	memset( wk, 0x00, sizeof(wk) );
	memcpy( wk, (const char*)buf, strlen((const char*)buf) );

	//! カンマでCSVのデータを分割する。
	split[0] = ',';	
	split[1] = '\0';	
	for( int i  = 0; i <= strlen((const char*)wk); i++ ){
		if( (wk[i] == ',' ) || (i == strlen((const char*)wk) ) ){
			if( len >  0 ){ 
				memset( &csv_data[pos],0x00, offset );
				memcpy( &csv_data[pos],&wk[start],len );
				if( strlen( (char*)rowInfoStr ) == 0 ){
					sprintf((char*)rowInfoStr,"%s",&csv_data[pos]);
				}else{
					sprintf((char*)rowInfoStr,"%s,%s",rowInfoStr,&csv_data[pos]);
				}				
			}else{
				memset( &csv_data[pos],0x00, offset );
				sprintf((char*)rowInfoStr,"%s,",rowInfoStr);
			} 
			len = 0;
			start  = i +1;
			cols++;						
			pos+=offset;
			
		}else{
			len++;
		}
		
	}
	pos = offset;

	for( i = 1,j=0; i < cols; i++,j++  ){	
		data[j] = str2time( &csv_data[pos] );
		pos+=offset;
	}
	
	return cols;
}
//*****************************************************************************************
/*!
 *	@fn collect_proctime_all
 *	@brief 集計の計算処理を実施する関数
 *	@param[in] procTimeArray : なし
 *	@return なし
 *	@details
 *   ・時間の集計処理を実施する。
 *	@author
 *	@par
 *	@throws
 *	@date
 *	  2017.01.25 新規作成 及川陽樹<br>
 */
//*****************************************************************************************
int calc_proctime_all( unsigned long *time, unsigned long  *max, unsigned long *min, 
									  unsigned long  val1, unsigned long  val2 ){

	/* -1.0以外の場合は、最大と最小を計算する。*/
	if( (val1  != -1 ) && (val1 != 0)){	
		if(  (*min)  > val1 )(*min) = val1;
		if(  (*max) <  val1 )(*max) =  val1;
	}	

	if( (val2  != -1 ) &&(val2 != 0) ){	
		if(  (*min)  > val2 ) (*min) = val2;
		if(  (*max) <  val2 )(*max) = val2;
	}	
	(*time) = (*max) - (*min);

	return 0;
}
//*****************************************************************************************
/*!
 *	@fn collect_proctime_all
 *	@brief 集計の計算処理を実施する関数
 *	@param[in] procTimeArray : なし
 *	@return なし
 *	@details
 *   ・時間の集計処理を実施する。
 *	@author
 *	@par
 *	@throws
 *	@date
 *	  2017.01.25 新規作成 及川陽樹<br>
 */
//*****************************************************************************************
int collect_proctime_all( double *max, double *min, double *sum, double *avg, unsigned long *cnt, double *val ){

	/* -1.0以外の場合は、最大と最小を計算する。*/
	if( (*val)  != -1.0 ){	
		(*sum) +=  (*val);
		(*cnt)++;

		if( (*min)  > (*val) ){
			 (*min) = (*val);
		}
		if( (*max) < (*val) ) (*max) = (*val);
		(*avg) = (*sum)/(*cnt);
	}

	return 0;
}
//*****************************************************************************************
/*!
 *	@fn collect_all
 *	@brief 集計の計算処理を実施する関数
 *	@param[in] procTimeArray : なし
 *	@return なし
 *	@details
 *   ・時間の集計処理を実施する。
 *	@author
 *	@par
 *	@throws
 *	@date
 *	  2017.01.25 新規作成 及川陽樹<br>
 */
//*****************************************************************************************
int collect_all(  double *procTimeArray, unsigned long rows){

	double *procTimeArray_wk = procTimeArray;
	
	for( int i = 0; i < 32; i++ ){	
			g_all_procTime_min[i] = 999;
	}
	
	for( int i = 0; i < rows; i++ ){	
		/* image_raw処理時間 */
		collect_proctime_all( &g_all_procTime_max[0], &g_all_procTime_min[0],&g_all_procTime_sum[0],
											 &g_all_procTime_avg[0], &g_all_procTime_cnt[0], &procTimeArray_wk[0] );
		/* image_raw通信時間 */
		collect_proctime_all( &g_all_procTime_max[1], &g_all_procTime_min[1],&g_all_procTime_sum[1],
											 &g_all_procTime_avg[1], &g_all_procTime_cnt[1], &procTimeArray_wk[1] );

		/* image_raw購読〜orb_descriptor配信まで */
		collect_proctime_all( &g_all_procTime_max[2], &g_all_procTime_min[2],&g_all_procTime_sum[2],
											 &g_all_procTime_avg[2], &g_all_procTime_cnt[2], &procTimeArray_wk[2] );

		/* orb_descriptor通信時間 */		
		collect_proctime_all( &g_all_procTime_max[3], &g_all_procTime_min[3],&g_all_procTime_sum[3],
											 &g_all_procTime_avg[3], &g_all_procTime_cnt[3], &procTimeArray_wk[3] );

		/* orb_descriptor購読〜tracker_state配信まで(Tracking処理時間) */
		collect_proctime_all( &g_all_procTime_max[4], &g_all_procTime_min[4],&g_all_procTime_sum[4],
											 &g_all_procTime_avg[4], &g_all_procTime_cnt[4], &procTimeArray_wk[4] );

		//! tracker_state通信時間
		collect_proctime_all( &g_all_procTime_max[5], &g_all_procTime_min[5],&g_all_procTime_sum[5],
											 &g_all_procTime_avg[5], &g_all_procTime_cnt[5], &procTimeArray_wk[5] );

		//! tracker_state処理時間
		collect_proctime_all( &g_all_procTime_max[6], &g_all_procTime_min[6],&g_all_procTime_sum[6],
											 &g_all_procTime_avg[6], &g_all_procTime_cnt[6], &procTimeArray_wk[6] );
		/* LocalMapping待ち時間 */
		collect_proctime_all( &g_all_procTime_max[7], &g_all_procTime_min[7],&g_all_procTime_sum[7],
											 &g_all_procTime_avg[7], &g_all_procTime_cnt[7], &procTimeArray_wk[7] );

		/* LocalMapping処理時間*/
		collect_proctime_all( &g_all_procTime_max[8], &g_all_procTime_min[8],&g_all_procTime_sum[8],
											 &g_all_procTime_avg[8], &g_all_procTime_cnt[8], &procTimeArray_wk[8] );

		/* LoopClosing待ち時間 */
		collect_proctime_all( &g_all_procTime_max[9], &g_all_procTime_min[9],&g_all_procTime_sum[9],
											 &g_all_procTime_avg[9], &g_all_procTime_cnt[9], &procTimeArray_wk[9] );

		/* LoopClosing処理時間 */
		collect_proctime_all( &g_all_procTime_max[10], &g_all_procTime_min[10],&g_all_procTime_sum[10],
											 &g_all_procTime_avg[10], &g_all_procTime_cnt[10], &procTimeArray_wk[10] );

		/* image_raw取得〜trackingまでの遅延時間 */
		collect_proctime_all( &g_all_procTime_max[11], &g_all_procTime_min[11],&g_all_procTime_sum[11],
											 &g_all_procTime_avg[11], &g_all_procTime_cnt[11], &procTimeArray_wk[11] );

		/* image_raw取得〜trackingまでの遅延時間+LocalMapping遅延時間 */
		collect_proctime_all( &g_all_procTime_max[12], &g_all_procTime_min[12],&g_all_procTime_sum[12],
											 &g_all_procTime_avg[12], &g_all_procTime_cnt[12], &procTimeArray_wk[12] );
											 
		/* image_raw取得〜trackingまでの遅延時間+LocalMapping遅延時間+LoopClosing遅延時間 */
		collect_proctime_all( &g_all_procTime_max[13], &g_all_procTime_min[13],&g_all_procTime_sum[13],
											 &g_all_procTime_avg[13], &g_all_procTime_cnt[13], &procTimeArray_wk[13] );

		/* 1フレーム当たりの遅延 */
		collect_proctime_all( &g_all_procTime_max[14], &g_all_procTime_min[14],&g_all_procTime_sum[14],
											 &g_all_procTime_avg[14], &g_all_procTime_cnt[14], &procTimeArray_wk[14] );
		
		procTimeArray_wk+=32;
	}

	return 0;
}
//*****************************************************************************************
/*!
 *	@fn collect
 *	@brief 集計処理を実施する関数
 *	@param[in] void    : なし
 *	@return なし
 *	@details
 *   ・分割3の集計処理を実施する。
 *	@author
 *	@par
 *	@throws
 *	@date
 *	  2017.01.25 新規作成 及川陽樹<br>
 */
//*****************************************************************************************
int collect( unsigned long *timeArray, double *procTimeArray,unsigned char *rowInfoStr){

	//!  image_rawの処理時間を計算する。 
	if( ( timeArray[0] != -1 ) && ( timeArray[1] != -1 ) ){  
		if( timeArray[0] > timeArray[1] ){
			procTimeArray[0] = (timeArray[0]  -	timeArray[1]) / 1000.0;
		}else{
			procTimeArray[0] = (timeArray[1] - 	timeArray[0])  / 1000.0;
		}
		calc_proctime_all( &g_all_proctime, &g_all_proctime_max, &g_all_proctime_min, timeArray[0], timeArray[1] );
	}else{
		procTimeArray[0] = -1.0;
	}
	
	//!  image_rawの通信時間を計算する。
	if( ( timeArray[1] != -1 ) && ( timeArray[2] != -1 ) ){  
			if( timeArray[1] > timeArray[2] ){
				procTimeArray[1]  = (timeArray[1] - timeArray[2])  / 1000.0;
			}else{
				procTimeArray[1] = (timeArray[2] - 	timeArray[1])  / 1000.0;
			}
			calc_proctime_all( &g_all_proctime, &g_all_proctime_max, &g_all_proctime_min, timeArray[1], timeArray[2] );
	}else{
		procTimeArray[1] = -1.0;
	}

	//!  image_raw購読〜 orb_descriptor配信まで	
	if( ( timeArray[2] != -1 ) && ( timeArray[3] != -1 ) ){  
		if( timeArray[2] > timeArray[3] ){
			procTimeArray[2]  = (timeArray[2] - timeArray[3])  / 1000.0;
		}else{
			procTimeArray[2] = (timeArray[3] - timeArray[2])  / 1000.0;
		}
		calc_proctime_all( &g_all_proctime, &g_all_proctime_max, &g_all_proctime_min, timeArray[2], timeArray[3] );
	}else{
		procTimeArray[2] = -1.0;
		g_image_raw_lost++;
	}
	
	//! orb_descriptor通信時間 
	if( ( timeArray[3] != -1 ) && ( timeArray[6] != -1 ) ){  
		if( timeArray[3] > timeArray[6] ){
			procTimeArray[3]  = (timeArray[3] - timeArray[6])  / 1000.0;
		}else{
			procTimeArray[3] = (timeArray[6] - timeArray[3])  / 1000.0;
		}
		calc_proctime_all( &g_all_proctime, &g_all_proctime_max, &g_all_proctime_min, timeArray[3], timeArray[6] );
	}else{
		procTimeArray[3] = -1.0;
	}		

	//! orb_descriptor購読〜tracker_state配信まで(Tracking処理時間)
	if( ( timeArray[6] != -1 ) && ( timeArray[7] != -1 ) ){  
		if( timeArray[6] > timeArray[7] ){
			procTimeArray[4]  = (timeArray[6] - timeArray[7])  / 1000.0;
		}else{
			procTimeArray[4] = (timeArray[7] - timeArray[6])  / 1000.0;
		}
		calc_proctime_all( &g_all_proctime, &g_all_proctime_max, &g_all_proctime_min, timeArray[6], timeArray[7] );
	}else{
		procTimeArray[4] = -1.0;
		g_orb_descriptor_lost++;
	}

	//! tracker_state通信時間
	if( ( timeArray[4] != -1 ) && ( timeArray[7] != -1 ) ){  
		if( timeArray[4] > timeArray[7] ){
			procTimeArray[5]  = (timeArray[4] - timeArray[7])  / 1000.0;
		}else{
			procTimeArray[5] = (timeArray[7] - timeArray[4])  / 1000.0;
		}
		calc_proctime_all( &g_all_proctime, &g_all_proctime_max, &g_all_proctime_min, timeArray[4], timeArray[7] );
	}else{
		procTimeArray[5] = -1.0;
	}
		
	//! tracker_state処理時間
	if( ( timeArray[4] != -1 ) && ( timeArray[5] != -1 ) ){  
		if( timeArray[4] > timeArray[5] ){
			procTimeArray[6]  = (timeArray[4] - timeArray[5])  / 1000.0;
		}else{
			procTimeArray[6] = (timeArray[5] - timeArray[4])  / 1000.0;
		}
		calc_proctime_all( &g_all_proctime, &g_all_proctime_max, &g_all_proctime_min, timeArray[4], timeArray[5] );
 	}else{
		procTimeArray[6] = -1.0;
		g_tracker_state_lost++;
	}
		
	//!  LocalMapping待ち時間
	if( ( timeArray[7] != -1 ) && ( timeArray[8] != -1 ) ){  
		if( timeArray[7] > timeArray[8] ){
			procTimeArray[7]  = (timeArray[7] - timeArray[8])  / 1000.0;
		}else{
			procTimeArray[7] = (timeArray[8] - timeArray[7])  / 1000.0;
		}
		calc_proctime_all( &g_all_proctime, &g_all_proctime_max, &g_all_proctime_min, timeArray[7], timeArray[8] );
 	}else{
		procTimeArray[7] = -1.0;
	}
	
	//! LocalMapping処理時間
	if( ( timeArray[8] != -1 ) && ( timeArray[9] != -1 ) ){  
		if( timeArray[8] > timeArray[9] ){
			procTimeArray[8]  = (timeArray[8] - timeArray[9])  / 1000.0;
		}else{
			procTimeArray[8] = (timeArray[9] - timeArray[8])  / 1000.0;
		}
		calc_proctime_all( &g_all_proctime, &g_all_proctime_max, &g_all_proctime_min, timeArray[8], timeArray[9] );
	}else{
		procTimeArray[8] = -1.0;
		g_localmapping_lost++;
	}
	
	//! LoopClosing待ち時間
	if( ( timeArray[9] != -1 ) && ( timeArray[10] != -1 ) ){  
		if( timeArray[9] > timeArray[10] ){
			procTimeArray[9]  = (timeArray[9] - timeArray[10])  / 1000.0;
		}else{
			procTimeArray[9] = (timeArray[10] - timeArray[9])  / 1000.0;
		}
		calc_proctime_all( &g_all_proctime, &g_all_proctime_max, &g_all_proctime_min, timeArray[9], timeArray[10] );
	}else{
		procTimeArray[9] = -1.0;
	}

	//! LoopClosing処理時間
	if( ( timeArray[10] != -1 ) && ( timeArray[11] != -1 ) ){  
		if( timeArray[10] > timeArray[11] ){
			procTimeArray[10]  = (timeArray[10] - timeArray[11])  / 1000.0;
		}else{
			procTimeArray[10] = (timeArray[11] - timeArray[10])  / 1000.0;
		}
		calc_proctime_all( &g_all_proctime, &g_all_proctime_max, &g_all_proctime_min, timeArray[10], timeArray[11] );
		//! キーフレームが処理される間隔を導く。
		if( g_prev_keyframe_time != 0 && ( timeArray[11] != 0 ) ){

			g_keyframe_proc_time = timeArray[11] - g_prev_keyframe_time;  
			g_keyframe_proc_time_sum += g_keyframe_proc_time;
			g_keyframe_proc_time_cnt++;
			g_keyframe_proc_time_avg  = g_keyframe_proc_time_sum / g_keyframe_proc_time_cnt;
			if( g_keyframe_proc_time_max < g_keyframe_proc_time ){
				g_keyframe_proc_time_max = g_keyframe_proc_time;
			}
			if( g_keyframe_proc_time_min > g_keyframe_proc_time ){
				g_keyframe_proc_time_min = g_keyframe_proc_time;
			}
			g_prev_keyframe_time  = timeArray[11];
	    } else{
	    	if( timeArray[11] != 0  ){
		    	g_prev_keyframe_time = timeArray[11];
		    }
	    }
	}else{
		procTimeArray[10] = -1.0;
		g_loopclosing_lost++;
	}

	//! image_raw取得〜trackingまでの遅延時間・・・(1)
	procTimeArray[11] =	((procTimeArray[0]==-1.0)?0.0:procTimeArray[0]) +
											((procTimeArray[1]==-1.0)?0.0:procTimeArray[1]) +
											((procTimeArray[2]==-1.0)?0.0:procTimeArray[2]) +
											((procTimeArray[3]==-1.0)?0.0:procTimeArray[3]) +
											((procTimeArray[4]==-1.0)?0.0:procTimeArray[4]);
									
	//! (1) + LocalMapping遅延時間	 ・・・(2)									
	procTimeArray[12] =	procTimeArray[11] + 
											((procTimeArray[7]==-1.0)?0.0:procTimeArray[7]) +
											((procTimeArray[8]==-1.0)?0.0:procTimeArray[8]);
	  
	//! (2) + LoopClosing遅延時間 ・・・(3)										
	procTimeArray[13] =	procTimeArray[12] + 
											((procTimeArray[9]==-1.0)?0.0:procTimeArray[9]) +
											((procTimeArray[10]==-1.0)?0.0:procTimeArray[10]);


	//! 1フレームの処理時間										
	for( int i = 0; i < 11; i++ ){
		if( procTimeArray[i] != -1.0 ){
			procTimeArray[14] += procTimeArray[i];
		}
	}
	for( int i = 0; i < 15; i++ ){
		if( procTimeArray[i] != -1.0){
			sprintf((char*)rowInfoStr,"%s,%lf",rowInfoStr,procTimeArray[i]);
		}else{
			sprintf((char*)rowInfoStr,"%s,",rowInfoStr);
		}
	}
	return 0;
	 
 }
//*****************************************************************************************
/*!
 *	@fn CollectFIleOut
 *	@brief 集計ファイルの出力処理を実施する。
 *	@param[in] void    : なし
 *	@return なし
 *	@details
 *   ・集計ファイルの出力処理を実施する。
 *	@author
 *	@par
 *	@throws
 *	@date
 *	  2017.01.25 新規作成 及川陽樹<br>
 */
//*****************************************************************************************
void CollectFIleOut(unsigned char *folder_name, unsigned long rows ){
	unsigned long hour;
	unsigned long min;
	unsigned long sec;
	unsigned long milli;
	unsigned long work;
	unsigned long detect;
	unsigned long lost;
	char fname[256];
	char tstr[256];
	char str[1024];
	std::ofstream wstream;
	struct tm tm;
	time_t t; 
	int ret = 0;
	
	work = g_all_proctime; 
 	hour = work / 3600000;
	work = work % 3600000;
	min   = work / 60000;
	work = work % 60000;
	sec    = work / 1000;
	milli  = work % 1000;

	g_loopclosing_lost++;
	g_localmapping_lost++;

	t = time(NULL);
	localtime_r(&t,&tm);

	//! ファイル名称を作成する。
	strftime( (char*)tstr,sizeof(str),"%Y%m%d-%H%M%S",&tm);
	sprintf( (char*)fname,"%s/%s_collect.csv",(char*)folder_name,(char*)tstr );
	
	//!  ファイルオープンを実施する。			
	wstream.open( (const char*)fname, std::ios::out | std::ios::app );
	
	memset(str,0x00,sizeof(str));
	sprintf(str,"All process time,%lu:%lu:%lu.%lu \n",hour,min,sec,milli );	
	wstream << str << std::endl;

	memset(str,0x00,sizeof(str));
	sprintf(str,"Topic,Detected,Lost\n");
	wstream << str ;	 

	memset(str,0x00,sizeof(str));
	sprintf(str,"Image-raw,%lu,%lu \n",rows-g_image_raw_lost, g_image_raw_lost );
	wstream << str ;	 

	memset(str,0x00,sizeof(str));
	sprintf(str,"Orb-descriptor,%lu,%lu \n",rows-g_orb_descriptor_lost, g_orb_descriptor_lost  );
	wstream << str ;	 

	memset(str,0x00,sizeof(str));
	sprintf(str,"Tracker-state,%lu,%lu \n",rows-g_tracker_state_lost,g_tracker_state_lost );
	wstream << str ;	 

	memset(str,0x00,sizeof(str));
	sprintf(str,"Local Mapping,%lu,%lu \n",rows-g_localmapping_lost,g_localmapping_lost );
	wstream << str ;	 

	memset(str,0x00,sizeof(str));
	sprintf(str,"Loop Closing,%lu,%lu \n",rows-g_loopclosing_lost,g_loopclosing_lost );
	wstream << str << std::endl;	 

	memset(str,0x00,sizeof(str));
	sprintf(str,"Measurment Points,max,min,average\n"); 
	wstream << str ;	 

	memset(str,0x00,sizeof(str));
	sprintf(str,"Image-Raw Read Time,%lf,%lf,%lf\n", 
			g_all_procTime_max[0],g_all_procTime_min[0],g_all_procTime_avg[0]);
	wstream << str ;	 

	memset(str,0x00,sizeof(str));
	sprintf(str,"Image-Raw Communication Time,%lf,%lf,%lf\n", 
			g_all_procTime_max[1],g_all_procTime_min[1],g_all_procTime_avg[1]);
	wstream << str ;	 

	memset(str,0x00,sizeof(str));
	sprintf(str,"Image-Raw Subscribe<->Orb_Descriptor Publish,%lf,%lf,%lf\n",
			 g_all_procTime_max[2],g_all_procTime_min[2],g_all_procTime_avg[2]);
	wstream << str ;	 

	memset(str,0x00,sizeof(str));
	sprintf(str,"Orb-Descriptor Communication Time,%lf,%lf,%lf\n", 
			g_all_procTime_max[3],g_all_procTime_min[3],g_all_procTime_avg[3]);
	wstream << str ;	 
	memset(str,0x00,sizeof(str));
	sprintf(str,"Orb-Descriptor Subscribe<->Tracker-State Publish,%lf,%lf,%lf\n",
			 g_all_procTime_max[4],g_all_procTime_min[4],g_all_procTime_avg[4]);
	wstream << str ;	 

	memset(str,0x00,sizeof(str));
	sprintf(str,"Tracker-State Communication Time,%lf,%lf,%lf\n",
			 g_all_procTime_max[5],g_all_procTime_min[5],g_all_procTime_avg[5]);
	wstream << str ;	 

	memset(str,0x00,sizeof(str));
	sprintf(str,"Tracker-State Process Time,%lf,%lf,%lf\n",
			 g_all_procTime_max[6],g_all_procTime_min[6],g_all_procTime_avg[6]);
	wstream << str ;	 

	memset(str,0x00,sizeof(str));
	sprintf(str,"LocalMapping Wait Time,%lf,%lf,%lf\n",
			 g_all_procTime_max[7],g_all_procTime_min[7],g_all_procTime_avg[7]);
	wstream << str ;	 

	memset(str,0x00,sizeof(str));
	sprintf(str,"LocalMapping Process Time,%lf,%lf,%lf\n",
			 g_all_procTime_max[8],g_all_procTime_min[8],g_all_procTime_avg[8]);
	wstream << str ;	 

	memset(str,0x00,sizeof(str));
	sprintf(str,"LoopClosing Wait Time,%lf,%lf,%lf\n",
			 g_all_procTime_max[9],g_all_procTime_min[9],g_all_procTime_avg[9]);
	wstream << str ;	 

	memset(str,0x00,sizeof(str));
	sprintf(str,"LoopClosing Process Time,%lf,%lf,%lf\n",
			 g_all_procTime_max[10],g_all_procTime_min[10],g_all_procTime_avg[10]);
	wstream << str ;	 

	memset(str,0x00,sizeof(str));
	sprintf(str,"Image-Raw Read<->Tracking,%lf,%lf,%lf\n",
			 g_all_procTime_max[11],g_all_procTime_min[11],g_all_procTime_avg[11]);
	wstream << str ;	 

	memset(str,0x00,sizeof(str));
	sprintf(str,"Image-Raw Read<->LocalMapping,%lf,%lf,%lf\n",
			 g_all_procTime_max[12],g_all_procTime_min[12],g_all_procTime_avg[12]);
	wstream << str ;	 

	memset(str,0x00,sizeof(str));
	sprintf(str,"Image-Raw Read<->LoopClosing,%lf,%lf,%lf\n",
			 g_all_procTime_max[13],g_all_procTime_min[13],g_all_procTime_avg[13]);
	wstream << str;	 

	memset(str,0x00,sizeof(str));
	sprintf(str,"1 Frame Process Time,%lf,%lf,%lf\n",
			 g_all_procTime_max[14],g_all_procTime_min[14],g_all_procTime_avg[14]);
	wstream << str  << std::endl;	 

	memset(str,0x00,sizeof(str));
	sprintf(str,"Keyframe Detect Average, %lf \n",(g_keyframe_proc_time_avg / 1000.0));
	wstream << str ;	 

	memset(str,0x00,sizeof(str));
	sprintf(str,"Keyframe Detect Max,%lf \n",(g_keyframe_proc_time_max / 1000.0));
	wstream << str ;	 

	memset(str,0x00,sizeof(str));
	sprintf(str,"Keyframe Detect Min,%lf \n",(g_keyframe_proc_time_min / 1000.0));
	wstream << str ;	 
	
	wstream.close();
	return;	
}
//*****************************************************************************************
/*!
 *	@fn csvOperation
 *	@brief csvファイルの操作を実施する。
 *	@param[in] void    : なし
 *	@return なし
 *	@details
 *   ・該当csvファイルをオープンし、ファイルの入力を実施する。
 *	@author
 *	@par
 *	@throws
 *	@date
 *	  2017.01.25 新規作成 及川陽樹<br>
 */
//*****************************************************************************************
int csvOperation( unsigned char* folder_name, unsigned char* file_name ){

	std::string  buffer;
	unsigned char fname[256];
	char outfname[256];
	std::ifstream istream;
	std::ofstream wstream;
	int ret = 0;
	int rows = 0;
	unsigned char rowInfoStr[2048];
	char tstr[256];
	char str[2048];
	struct tm tm;
	time_t t; 

	memset( fname, 0x00, sizeof(fname) );
	sprintf( (char*)fname, "%s/%s",folder_name,file_name);
	
	t = time(NULL);
	localtime_r(&t,&tm);

	//! ファイル名称を作成する。
	strftime( (char*)tstr,sizeof(tstr),"%Y%m%d-%H%M%S",&tm);
	sprintf( (char*)outfname,"%s/%s_raws.csv",(char*)folder_name,(char*)tstr );
	
	//!  ファイルオープンを実施する。			
	wstream.open( (const char*)outfname, std::ios::out | std::ios::app );

	//!  ファイルオープンを実施する。			
	istream.open( (const char*)fname,  (std::ios::in | std::ios::app) );
	memset(str,0x00,sizeof(str));
	sprintf(str,"ID,Image-raw read,,Image-row Subscribe<=>Orb-destractor Publish,,Tracker-State Proc,,Orb-destractor Subscribe<=> Track State Publish,,LocalMapping,,LoopClosing,,Image-raw ReadTime,Image-raw Communication Time,Image-raw Subscribe<=>Orb-destractor Publish,Orb-destracotr Communication Time,Orb-destractor Subscribe<=>Tracker-State Publish,Tracker-State Communication Time,Tracker-State ProcTime,LocalMapping WaitTime,LocalMapping ProcTime,LoopClosing WaitTime,LoopClosing ProcTime,Image-raw Read<=>Tracking,Image-raw Read<=>LocalMapping,Image-raw Read<=>LoopClosing,1 Frame Proc Time");
	wstream << str << std::endl;
	while ( !istream.eof() ){
		memset( rowInfoStr, 0x00, sizeof(rowInfoStr));
		//! ファイルの読み出しを行う。
		std::getline(istream, buffer);

		//! 	csvファイルの分割処理を実施する。
		splitcsv( (unsigned char*)buffer.c_str(), (unsigned char*)&g_timeStrArray[rows][0], &g_timeArray[rows][0], 256,rowInfoStr);		
		//! 分割の集計処理を実施する。
		collect( &g_timeArray[rows][0],  &g_procTimeArray[rows][0],rowInfoStr);
		if( strlen((char*)&g_timeStrArray[rows][0]) != 0 ){
			wstream << rowInfoStr << std::endl;
		}
		rows++;
		
	}	
	rows--;
	collect_all( (double*)g_procTimeArray, rows );
	//! 入力ファイルをクローズする。
	istream.close();
	wstream.close();
	
	CollectFIleOut((unsigned char*)folder_name,rows);
	return ret;				

}
//*****************************************************************************************
/*!
 *	@fn main
 *	@brief csvファイルの入力操作を実施するメイン関数
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

	csvOperation((unsigned char*)argv[1],(unsigned char*)argv[2]);				

	return 0;
}

