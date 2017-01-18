#include "dataset_reader_node.cpp"
#ifdef ENABLE_PERFORM
#include "measurmentManager.h"
#endif

#ifdef ENABLE_PERFORM
extern measurmentManager *	g_measurmentServer;
extern char* __progname;
#endif

class TumReader:  public DatasetReader{

  virtual int LoadImages(const string &dataPath,
                          vector<string>& vstrImageFilenames,
                          vector<double>& vTimestamps
                          ){
    ifstream f;

    f.open((dataPath + "/rgb.txt").c_str());
    if (f.fail()){
      printf("open failed[%s, %s]\n", dataPath.c_str(), "/rgb.txt");
      return true;
    }
    // skip first three lines
    string s0;
    getline(f,s0);
    getline(f,s0);
    getline(f,s0);
    
    double base = -1.0;

    while(!f.eof())
    {
        string s;
        getline(f,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            string sRGB;
            ss >> t;
            if (base < 0)
              base = t;

            vTimestamps.push_back(t-base);
            
            ss >> sRGB;
            vstrImageFilenames.push_back(sRGB);
        }
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
  ros::init(argc, argv, "tum_reader");

#ifdef ENABLE_PERFORM
  g_measurmentServer = new measurmentManager();
  memset( (char*)measument_path,0x00, sizeof(measument_path)); 
  sprintf( (char*)measument_path,"/%s/measurment",__progname);
  ROS_ERROR("%s",measument_path);
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
