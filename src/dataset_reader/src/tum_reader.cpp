#include "dataset_reader_node.cpp"

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

  if(argc < 2)
    {
      cerr << endl << "Usage: ./dataset_reader_node path_to_sequence" << endl;
      return 1;
    }

  std::string data_path(argv[1]);
  printf("data path: %s.\n", data_path.c_str()) ;
  ros::init(argc, argv, "tum_reader");
  TumReader tum_reader;
  tum_reader.run(data_path);
  return 0;
}
