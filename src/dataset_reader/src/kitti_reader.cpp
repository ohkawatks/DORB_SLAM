#include "dataset_reader_node.cpp"
#include<iomanip>
#ifdef ENABLE_PERFORM
#include "measurmentManager.h"
#endif

#ifdef ENABLE_PERFORM
extern measurmentManager *	g_measurmentServer;
extern char* __progname;
#endif

//const string &dataPath,
class TumReader:  public DatasetReader{
  virtual int LoadImages(const string &strPathToSequence, 
                         vector<string> &vstrImageFilenames, 
                         vector<double> &vTimestamps)
  {
    ifstream fTimes;
    string strPathTimeFile = strPathToSequence + "/times.txt";
    fTimes.open(strPathTimeFile.c_str());
    if (fTimes.fail()){
      return true;
    }
    while(!fTimes.eof())
      {
        string s;
        getline(fTimes,s);
        if(!s.empty())
          {
            stringstream ss;
            ss << s;
            double t;
            ss >> t;
            vTimestamps.push_back(t);
          }
      }

    string strPrefixLeft = "/image_0/";

    const int nTimes = vTimestamps.size();
    vstrImageFilenames.resize(nTimes);

    for(int i=0; i<nTimes; i++)
      {
        stringstream ss;
        ss << setfill('0') << setw(6) << i;
        vstrImageFilenames[i] = strPrefixLeft + ss.str() + ".png";
      }
    return false;
  }
};


int main(int argc, char **argv)
{
#ifdef ENABLE_PERFORM
   unsigned char measument_path[256];
#endif

  if(argc < 2)
    {
      cerr << endl << "Usage: ./dataset_reader_node path_to_sequence" << endl;
      return 1;
    }

  std::string data_path(argv[1]);
  printf("data path: %s.\n", data_path.c_str()) ;
  ros::init(argc, argv, "tum_reader");
#ifdef ENABLE_PERFORM
   g_measurmentServer = new measurmentManager();
#endif

#ifdef ENABLE_PERFORM
  memset( (char*)measument_path,0x00, sizeof(measument_path)); 
  sprintf( (char*)measument_path,"/%s/measurment",__progname);
#endif

  ros::NodeHandle nodeHandler;
#ifdef ENABLE_PERFORM
  ros::ServiceServer server = nodeHandler.advertiseService( (char*)measument_path, 
																					&measurmentManager::service, g_measurmentServer);
#endif

  TumReader tum_reader;
  tum_reader.run(data_path);
  return 0;
}
