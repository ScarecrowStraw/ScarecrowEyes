#include <iostream>
#include <pthread.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>

using namespace cv;
using namespace std;

static pthread_mutex_t foo_mutex = PTHREAD_MUTEX_INITIALIZER;

struct thread_data
{
    std::string path;
    int thread_id;
    std::string window_title;  
};

void *capture(void *threadarg)
{
    struct thread_data *data;
    data = (struct thread_data *) threadarg;

    cv::VideoCapture cap;

    // Safely open video stream
    pthread_mutex_lock(&foo_mutex);
    cap.open(data->path, cv::CAP_GSTREAMER);
    pthread_mutex_unlock(&foo_mutex);

    if (!cap.isOpened())
    {
        std::cout << "Camera " << data->thread_id << " open failed" << std::endl;
    }

    std::cout << "Camera " << data->thread_id << " open successed" << std::endl;

    cv::Mat frame;
    string ext = ".jpg";
    std::string result;

    // Create window with unique title
    cv::namedWindow(data->window_title);

    while(true)
    {
        cap >> frame;
        std::cout << "Thread " << data->thread_id << std::endl;
        cv::imshow(data->window_title, frame);
        cv::waitKey(10);
    }

    // Release VideoCapture object
    cap.release();
    // Destroy previously created window
    cv::destroyWindow(data->window_title);

    // Exit thread 
    pthread_exit(NULL);
}

int main(void)
{
    std::cout << "Junbot hello !!!" << std::endl;

    const int thread_count = 2;

    pthread_t threads[thread_count];
    struct thread_data td[thread_count];

    // Initialize thread data beforehand
    td[0].path = "nvarguscamerasrc sensor-id=0 ! video/x-raw(memory:NVMM), width=640, height=480, format=(string)NV12, framerate=(fraction)20/1 ! nvvidconv flip-method=0 ! video/x-raw, width=640, height=480, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";
    td[0].window_title = "First Window";
    td[1].path = "nvarguscamerasrc sensor-id=1 ! video/x-raw(memory:NVMM), width=640, height=480, format=(string)NV12, framerate=(fraction)20/1 ! nvvidconv flip-method=0 ! video/x-raw, width=640, height=480, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";
    td[1].window_title = "Second Window";


    int rc=0;
    for( int i = 0; i < thread_count; i++ ) 
    {
        cout <<"main() : creating thread, " << i << endl;
        td[i].thread_id = i;

        rc = pthread_create(&(threads[i]), NULL, capture, (void *)& (td[i]) );

        if (rc) 
        {
            cout << "Error:unable to create thread," << rc << endl;
            exit(-1);
        }
    }

    //Wait for the previously spawned threads to complete execution
    for( int i = 0; i < thread_count; i++ )
        pthread_join(threads[i], NULL);

    pthread_exit(NULL);

    return 0;
}
