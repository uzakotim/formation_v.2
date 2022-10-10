#ifndef MY_CLASS
#define MY_CLASS
#include <nodelet/nodelet.h>

#include <ros/ros.h>
#include <ros/package.h>

namespace cameras_reader
{

    class NodeletClass : public nodelet::Nodelet
    {
        public:
            virtual void onInit();
    };

}
#endif