/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<iomanip>

#include<opencv2/core/core.hpp>

#include"System.h"

#include <glob.h> // glob(), globfree()
#include <string.h> // memset()
#include <vector>
#include <stdexcept>
#include <string>
#include <sstream>

#include <sys/stat.h>

using namespace std;

void LoadImages(const string &strSequence, vector<string> &vstrImageFilenames,
                vector<double> &vTimestamps);

vector<string> glob(const string& pattern) {

    // glob struct resides on the stack
    glob_t glob_result;
    memset(&glob_result, 0, sizeof(glob_result));

    // do the glob operation
    int return_value = glob(pattern.c_str(), GLOB_TILDE, NULL, &glob_result);
    if(return_value != 0) {
        globfree(&glob_result);
        stringstream ss;
        ss << "glob() failed with return_value " << return_value << endl;
        throw std::runtime_error(ss.str());
    }

    // collect all the filenames into a std::list<std::string>
    vector<string> filenames;
    for(size_t i = 0; i < glob_result.gl_pathc; ++i) {
        filenames.push_back(string(glob_result.gl_pathv[i]));
    }

    // cleanup
    globfree(&glob_result);

    // done
    return filenames;
}

bool IsPathExist(const std::string &s)
{
  struct stat buffer;
  return (stat (s.c_str(), &buffer) == 0);
}

int main(int argc, char **argv)
{

    // const string path = "/home/users/trn_ak/*";
    // vector<string> filenames = glob(path);

/**
    if(argc != 4)
    {
        cerr << endl << "Usage: ./mono_kitti path_to_vocabulary path_to_settings path_to_sequence" << endl;
        return 1;
    }
**/

    // const string sequenceDir = argv[3] + "/*";
    // const string sequenceDir;
    // vector<string> bagSequences = glob(sequenceDir);

    

    vector<string> bagSequences = glob(argv[3]);

    for(int i=0; i<bagSequences.size(); i++)
    {

	// DEBUG
	// bagSequences[i] = "/share/projects/2019_kss_personal_jupyter_notebooks/trn_ak/sts_odom_dataset/paketzentrum_eifeltor/sequences/2019-05-13-13-54-41_hmm";

	cout << "Retrieving images from " << bagSequences[i] << endl;
	


	// Retrieve paths to images
	vector<string> vstrImageFilenames;
	vector<double> vTimestamps;
	LoadImages(string(argv[3]), vstrImageFilenames, vTimestamps);

	int nImages = vstrImageFilenames.size();

	// Create SLAM system. It initializes all system threads and gets ready to process frames.
	ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);

	// Vector for tracking time statistics
	vector<float> vTimesTrack;
	vTimesTrack.resize(nImages);

	cout << endl << "-------" << endl;
	cout << "Start processing sequence ..." << endl;
	cout << "Images in the sequence: " << nImages << endl << endl;

	// Main loop
	cv::Mat im;
	for(int ni=0; ni<nImages; ni++)
	{
		// Read image from file
		im = cv::imread(vstrImageFilenames[ni],CV_LOAD_IMAGE_UNCHANGED);
		double tframe = vTimestamps[ni];

		if(im.empty())
		{
		    cerr << endl << "Failed to load image at: " << vstrImageFilenames[ni] << endl;
		    return 1;
		}

		#ifdef COMPILEDWITHC11
		std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
		#else
		std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
		#endif

		// Pass the image to the SLAM system
		SLAM.TrackMonocular(im,tframe);

		#ifdef COMPILEDWITHC11
		std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
		#else
		std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
		#endif

		double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

		vTimesTrack[ni]=ttrack;

		// Wait to load the next frame
		double T=0;
		if(ni<nImages-1)
		    T = vTimestamps[ni+1]-tframe;
		else if(ni>0)
		    T = tframe-vTimestamps[ni-1];

		if(ttrack<T)
		    usleep((T-ttrack)*1e6);
	}

	// Stop all threads
	SLAM.Shutdown();

	// Tracking time statistics
	sort(vTimesTrack.begin(),vTimesTrack.end());
	float totaltime = 0;
	for(int ni=0; ni<nImages; ni++)
	{
		totaltime+=vTimesTrack[ni];
	}
	cout << "-------" << endl << endl;
	cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
	cout << "mean tracking time: " << totaltime/nImages << endl;

	// Save camera trajectory
	// SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");    

//	string bagPath = bagSequences[i];
	string bagKey = bagSequences[i].substr(bagSequences[i].find("sequences")+10);
// 	const char* cstr = str.c_str();
	// cout << bagKey << endl;
	string posePath = "/home/users/trn_ak/git_clones/orb_slam2/" + bagKey; 
	const char* cPosePath = posePath.c_str();
	const string keyFrameFile = posePath + "/KeyFrameTrajectory.txt"; 

	cout << cPosePath << endl;
	cout << keyFrameFile << endl;
        // const int dir_err = mkdir("foo", S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
        // const int dir_err = mkdir(cPosePath, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
	if (!IsPathExist(posePath))
	{
	    const int dir_err = mkdir(cPosePath, 0777);
	    if (dir_err == -1)
	    {
	        cout << "Error creating directory!" << endl;
	        return 1;
    	    }
   	} 
	// SLAM.SaveKeyFrameTrajectoryTUM("/home/users/trn_ak/git_clones/KeyFrameTrajectory.txt");    
	SLAM.SaveKeyFrameTrajectoryTUM(keyFrameFile);    

	return 0;

    }    



    return 0;
}

void LoadImages(const string &strPathToSequence, vector<string> &vstrImageFilenames, vector<double> &vTimestamps)
{
    ifstream fTimes;
    // DEBUG
    // KITTI dataset
    // string strPathTimeFile = strPathToSequence + "/times.txt";
    // sts dataset
    string strPathTimeFile = strPathToSequence + "/timestamps.txt";
    fTimes.open(strPathTimeFile.c_str());
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

    // DEBUG
    // string strPrefixLeft = strPathToSequence + "/image_0/";
    // KITTI dataset
    // string strPrefixLeft = strPathToSequence + "/image_2/";
    // sts dataset
    string strPrefixLeft = strPathToSequence + "/images/";

    const int nTimes = vTimestamps.size();
    vstrImageFilenames.resize(nTimes);

    for(int i=0; i<nTimes; i++)
    {
        stringstream ss;
    	// DEBUG
    	// KITTI dataset
        //ss << setfill('0') << setw(6) << i;
    	// sts dataset
        ss << setfill('0') << setw(10) << i;
    	// DEBUG
    	// KITTI dataset
        // vstrImageFilenames[i] = strPrefixLeft + ss.str() + ".png";
    	// sts dataset
        vstrImageFilenames[i] = strPrefixLeft + ss.str() + ".jpg";
    }
}
