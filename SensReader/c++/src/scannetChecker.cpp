/*
*  g++ scannetChecker.cpp -std=c++17 -lstdc++fs -o scannetChecker
*/
#include <string>
#include <iostream>
#include <filesystem>
#include "sensorData.h"
#include <tuple>

using namespace std;

enum TimestampStatus
{
    NOT_AVAILABLE,
    NOT_MONOTONIC,
    GOOD,
};

struct SeqStatus
{
    size_t invalid_pose_num;
    size_t valid_pose_num;
    TimestampStatus depth_ts;
    TimestampStatus color_ts;
};

/// input: .sens filename and logging level
/// output: invalid poses number, color timestamp status, depth timestamp status
void analyze_sens(const std::string& input, bool verbose, SeqStatus & seq)
{
    // Input
    if(verbose)
    {
        cout << "Loading data ... ";
        cout.flush();
    }
    ml::SensorData sd(input);
    if(verbose)
    {
        cout << "done!" << endl;
        cout << sd << endl;
    }

    // Stats
    bool ts_d_monotonic = true;
    bool ts_c_monotonic = true;
    bool ts_d_available = false;
    bool ts_c_available = false;
    uint64_t ts_d_last = 0;
    uint64_t ts_c_last = 0;
    size_t illegal_transformation = 0;

    // Iterating through all frames
    for (size_t i = 0; i < sd.m_frames.size(); i++) 
    {

        // Test timestamps
        const ml::SensorData::RGBDFrame& frame = sd.m_frames[i];
        uint64_t t_d = frame.getTimeStampDepth();
        uint64_t t_c = frame.getTimeStampColor();
        if (t_d > 0) ts_d_available = true;
        if (t_c > 0) ts_c_available = true;
        if (t_d < ts_d_last) ts_d_monotonic = false;
        if (t_c < ts_c_last) ts_c_monotonic = false;
        ts_d_last = t_d;
        ts_c_last = t_c;

        // Test poses
        ml::mat4f t = frame.getCameraToWorld();
        if(t.matrix[15] != 1 || t.matrix[14] != 0 || t.matrix[13] != 0 || t.matrix[12] != 0)
        {
            illegal_transformation++;
            if(verbose)
                cout << "Found illegal transformation at frame " << to_string(i) << ": ["
                     << t.matrix[0] << ", " << t.matrix[1] << ", " << t.matrix[2] << ", " <<t.matrix[3] << "]["
                     << t.matrix[4] << ", " << t.matrix[5] << ", " << t.matrix[6] << ", " <<t.matrix[7] << "]["
                     << t.matrix[8] << ", " << t.matrix[9] << ", " << t.matrix[10] << ", " <<t.matrix[11] << "]["
                     << t.matrix[12] << ", " << t.matrix[13] << ", " << t.matrix[14] << ", " <<t.matrix[15] << "]]" << endl;
        }
    }

    cout << "Depth timestamps are monotonic: " << (ts_d_monotonic ? "\x1B[32m yes" : "\x1B[31m no") << "\x1B[0m \n";
    cout << "RGB   timestamps are monotonic: " << (ts_c_monotonic ? "\x1B[32m yes" : "\x1B[31m no") << "\x1B[0m \n";
    cout << "Depth timestamps are available: " << (ts_d_available ? "\x1B[32m yes" : "\x1B[31m no") << "\x1B[0m \n";
    cout << "RGB   timestamps are available: " << (ts_c_available ? "\x1B[32m yes" : "\x1B[31m no") << "\x1B[0m \n";
    cout << "All  camera  poses  were legal: " << ( (illegal_transformation==0) ? "\x1B[32m yes " : "\x1B[31m no ") << illegal_transformation << "/" << sd.m_frames.size() << "\x1B[0m \n";
    cout << endl;

    seq.valid_pose_num = sd.m_frames.size();
    seq.invalid_pose_num = illegal_transformation;
    seq.depth_ts = ts_d_available ? (ts_d_monotonic ? GOOD : NOT_MONOTONIC): NOT_AVAILABLE;
    seq.color_ts = ts_c_available ? (ts_c_monotonic ? GOOD : NOT_MONOTONIC): NOT_AVAILABLE;
}

int main(int argc, char* argv[])
{
    if(argc < 2 || argc > 3) 
    {
        cout << "A tool to analyse scannet *.sens data.\n\n"
                "Error, invalid arguments.\n"
                "Mandatory: input *.sens file / input *.txt file\n"
                "Optional path to dataset dir"
             << endl;
        return 1;
    }

    // Parsing input data
    std::string filename = argv[1];
    if(filename.find_last_of("/")==filename.size()-1)
    {
        filename.pop_back();
    }
    
    // Analyse
    size_t no_sens_num = 0;
    size_t total_num = 0;
    size_t total_healthy_seq = 0;
    size_t total_pose_valid_seq = 0;
    size_t total_timestamp_valid_seq = 0;
    size_t total_pose_invalid_num = 0;
    size_t total_pose_valid_num = 0;

    if(filename.substr(filename.find_last_of("/") + 1) == "scans")
    {
        // Analyse the root folder of scenes
        for (const auto & entry : std::filesystem::directory_iterator(filename))
        {
            std::string sceneRoot = entry.path();
            std::string sensFileName = sceneRoot + "/" + sceneRoot.substr(sceneRoot.find_last_of("/") + 1) + ".sens";
            cout << "Processing" << sensFileName << endl;
            SeqStatus seq;
            if (std::filesystem::exists(sensFileName))
            {
                analyze_sens(sensFileName, false, seq);
            }
            else
            {
                cout<<"file missing, ignored.\n"<<endl;
                no_sens_num += 1;
                continue;
            }
            total_pose_valid_num += seq.valid_pose_num;
            total_pose_invalid_num += seq.invalid_pose_num;
            total_timestamp_valid_seq += (seq.depth_ts==GOOD)&&(seq.color_ts==GOOD) ? 1 : 0;
            total_pose_valid_seq += (seq.invalid_pose_num==0) ? 1 : 0;
            total_healthy_seq += (seq.invalid_pose_num==0)&&(seq.depth_ts==GOOD)&&(seq.color_ts==GOOD) ? 1 : 0;
            total_num += 1;
        }
    }
    else if(filename.substr(filename.find_last_of(".") + 1) == "sens")
    {
        // Analyse single .sens file inside a scene
        SeqStatus seq;
        analyze_sens(filename, true, seq);
    }
    else
    {
        // Only accept the root folder of scenes or .sens inside a scene
        cout<<"ERROR: wrong input name"<<endl;
        return 1;
    }

    // Report
    cout<<"======================"<<endl;
    cout<<"====    Report    ===="<<endl;
    cout<<"======================"<<endl;
    cout<<"No .sens file inside: "<<no_sens_num<<endl;
    cout<<"Total Valid Pose Number: "<<total_pose_valid_num<<endl;
    cout<<"Total Invalid Pose Number: "<<total_pose_invalid_num<<endl;
    cout<<"Total Valid Pose Seq Number: "<<total_pose_valid_seq<<" / "<<total_num<<endl;
    cout<<"Total Valid Timestamp Seq Number: "<<total_timestamp_valid_seq<<" / "<<total_num<<endl;
    cout<<"Total Healthy Seq Number: "<<total_healthy_seq<<" / "<<total_num<<endl;

    return 0;
}
