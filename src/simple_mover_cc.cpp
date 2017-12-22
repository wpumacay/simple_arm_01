
#include "ros/ros.h"
#include "std_msgs/Float64.h"


#include <iostream>
#include <string>
#include <cmath>
#include <vector>

using namespace std;


#define SMOVER_TOPIC_BASE_NAME "/simple_arm/joint_###_position_controller/command"
#define SMOVER_STR_REPLACE_PART "###"
#define SMOVER_NUM_JOINTS 2

#define _PI 3.14159

class LSimpleMover
{

	private :

	ros::NodeHandle m_nodeHandle;

	int m_numJoints;
	string m_topicBaseName;

	vector<ros::Publisher> m_jointPubs;

	double m_startTime;


	string _getJointTopic( int pJointIndx )
	{
		string _jointTopic = m_topicBaseName;

		auto _indx = _jointTopic.find( SMOVER_STR_REPLACE_PART );

		cout << "indx: " << _indx << endl;

		_jointTopic.replace( _indx, 3, to_string( pJointIndx ) );

		cout << "jointTopic: " << _jointTopic << endl;

		return _jointTopic;
	}


	public :


	LSimpleMover()
	{
		m_numJoints = SMOVER_NUM_JOINTS;
		m_topicBaseName = SMOVER_TOPIC_BASE_NAME;

		for ( int q = 0; q < m_numJoints; q++ )
		{
			auto _pub = m_nodeHandle.advertise<std_msgs::Float64>( _getJointTopic( q + 1 ), 1000 );
			m_jointPubs.push_back( _pub );
		}
	}

	~LSimpleMover() {}

	void run()
	{
		auto _rate = ros::Rate( 10 );

		m_startTime = ros::Time::now().toSec();

		while ( ros::ok() )
		{
			double _elapsed = ros::Time::now().toSec() - m_startTime;

			for ( ros::Publisher _jointPub : m_jointPubs )
			{
				std_msgs::Float64 _jointMsg;
				_jointMsg.data = sin( 2 * _PI * 0.1 * _elapsed ) * ( _PI / 2.0 );

				_jointPub.publish( _jointMsg );
			}

			_rate.sleep();
			ros::spinOnce();
		}
	}

};


int main( int argc, char** argv )
{

	ros::init( argc, argv, "arm_mover" );

	LSimpleMover _smover;

	_smover.run();

	return 0;
}



