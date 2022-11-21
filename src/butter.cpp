// Author:
/*
 * Node: butter.cpp
 * Author: Amir Hossein Ebrahimnezhad
 * Date: Nov 21/22
*/

// Libraries
#include <ros/ros.h>

#include <unistd.h>
#include <vector>

//Standard headers
#include <time.h>
#include <math.h>

//Custom Message Declaration 
#include <anafi_ros/spData.h>
#include <anafi_ros/relpos.h>
#include <fstream>

//

struct DataIndex
{
    int t;
    int x_anafi, y_anafi, z_anafi;
    int x_bebop, y_bebop, z_bebop;

    int x_rel, y_rel, z_rel;
    int x_est, y_est, z_est;

};


// Functions

std::vector<std::vector<std::string>> read_csv(std::string);

void  printVector(std::vector<float>, int); 
 
int delta_n(int);

// Main Code


class Butterworth
{
    
private: // Private Members
    

public: // Public Members

    int n; // Filter Order
    std::vector<float> a_, b_, x, y; // Filter Coefficients


    Butterworth()
    {

    };
    
    // Butterworth(int i, float *a, float *b): n(i)
    // {

    //     for(int  j = 0; j < n; j++)
    //     {
    //         a_.push_back(a[j]);
    //         b_.push_back(b[j]);

    //         x.push_back(0.0);
    //         y.push_back(0.0);
    //     }

    // };

    ~Butterworth()
    {

    };

    void init(int filt_order, float *a, float *b)
    {   
        n = filt_order;

        for(int  j = 0; j < n; j++)
        {
            a_.push_back(a[j]);
            b_.push_back(b[j]);

            x.push_back(0.0);
            y.push_back(0.0);
        }
    }


    float filter(float sample)
    {
        x = shift_array(sample, x);
        y = shift_array(0.0, y);

        float sum = 0.0;

        for (int  j = 1; j < n; j++)
        {
            sum += (b_[j]*x[j] - a_[j]*y[j]);
        }

        sum += b_[0]*x[0];

        y[0] = sum;

        return y[0];

    }

    std::vector<float> shift_array(float x0, std::vector<float> vector_)
    {
        for(int  j = n-1; j > 0; j--)
        {
            vector_[j] = vector_[j-1];
        }
        vector_[0] = x0;

        return vector_;
    }

};


class FilterAnalysis
{
    
private: // Private Members
    
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh;



public: // Public Members


    ros::Publisher pnp_raw_pub;
    ros::Publisher pnp_filtered_pub;

    DataIndex data_idx;

    anafi_ros::relpos pnp_raw, pnp_filtered;

    int filter_order;

    float b_x[3] = {0.0055,    0.0111,    0.0055};
    float a_x[3] = {1.0000,   -1.7786,    0.8008};

    float b_y[3] = {0.0055,    0.0111,    0.0055};
    float a_y[3] = {1.0000,   -1.7786,    0.8008};

    float b_z[3] = {0.0055,    0.0111,    0.0055};
    float a_z[3] = {1.0000,   -1.7786,    0.8008};

    float x_n;

    Butterworth filter_x, filter_y, filter_z;

    std::string csv_path = "/home/bluesky/catkin_ws/src/anafi_ros/src/repo/pursuit/pursuit_on_pnp_11_11_15_27/csv/pursuit_data.csv";
    std::vector<std::vector<std::string>> csv_data;

    FilterAnalysis()
    {

        filter_order = 3;

        data_idx.x_rel = 18;
        data_idx.y_rel = 19;
        data_idx.z_rel = 20;

        data_idx.x_est = 21;
        data_idx.y_est = 22;
        data_idx.z_est = 23;

        filter_x.init(filter_order, a_x, b_x);
        filter_y.init(filter_order, a_y, b_y);
        filter_z.init(filter_order, a_z, b_z);

        pnp_raw_pub = nh_.advertise<anafi_ros::relpos>("/anafi/relpos/pnp_raw", 1000);
        pnp_filtered_pub = nh_.advertise<anafi_ros::relpos>("/anafi/relpos/pnp_filtered", 1000);

        csv_data = read_csv(csv_path);

    };

    ~FilterAnalysis()
    {

    };

    void get_pnp(int idx)
    {

        pnp_raw.xrel = std::stod(csv_data[idx][data_idx.x_est]);
        pnp_raw.yrel = std::stod(csv_data[idx][data_idx.y_est]);
        pnp_raw.zrel = std::stod(csv_data[idx][data_idx.z_est]);
        
    }

    void filter_pnp(int idx)
    {

        get_pnp(idx);

        pnp_filtered.xrel = filter_x.filter(pnp_raw.xrel);
        pnp_filtered.yrel = filter_y.filter(pnp_raw.yrel);
        pnp_filtered.zrel = filter_z.filter(pnp_raw.zrel);

    }

    std::vector<std::vector<std::string>> read_csv(std::string filename)
    {

        std::vector<std::vector<std::string>> content;
        std::vector<std::string> row;
        std::string line, word;

        std::fstream file(filename, std::ios::in);
        if (file.is_open())
        {
            while (getline(file, line))
            {
                row.clear();

                std::stringstream str(line);

                while (getline(str, word, ','))
                    row.push_back(word);
                content.push_back(row);
            }
        }

        else
            std::cout << "Could not open the file\n";


        return content;

    }
};


int main(int argc, char **argv) {

    ros::init(argc, argv, "first_publisher_node");
    ROS_INFO("Filtering Initiated!\n");

    FilterAnalysis filter_analysis;

    ros::Rate rate(20);

    int c = 200;

    while(ros::ok()){

        filter_analysis.filter_pnp(c);

        filter_analysis.pnp_raw_pub.publish(filter_analysis.pnp_raw);
        filter_analysis.pnp_filtered_pub.publish(filter_analysis.pnp_filtered);

        rate.sleep();

        c++;
    }


    return 0;
}

void printVector(std::vector<float> vector_, int n)  
{  
    std::cout << "Printing vector elements:" << std::endl;  
    for (int i = 0; i < n; i++)  
    {  
                   std::cout<< vector_[i] <<"\t";    
    }  
    std::cout << "\n"; 
}  

int delta_n(int n)
{

    if (n == 0) return 1.0;
    else return 0.0;
}


