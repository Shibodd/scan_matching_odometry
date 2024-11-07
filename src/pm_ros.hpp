#ifndef PM_ROS_HPP
#define PM_ROS_HPP

// Credits to this guy https://github.com/norlab-ulaval/libpointmatcher_ros/tree/humble

#include <pointmatcher/PointMatcher.h>
#include <sensor_msgs/msg/laser_scan.hpp>

template<typename T>
typename PointMatcher<T>::DataPoints rosMsgToPointMatcherCloud(const sensor_msgs::msg::LaserScan& rosMsg)
{
	typedef PointMatcher <T> PM;
	typedef typename PM::DataPoints DataPoints;
	typedef typename DataPoints::Label Label;
	typedef typename DataPoints::Labels Labels;

	Labels featLabels;
	featLabels.push_back(Label("x", 1));
	featLabels.push_back(Label("y", 1));
	featLabels.push_back(Label("pad", 1));

	// Build descriptors
	Labels descLabels;
	if(!rosMsg.intensities.empty())
	{
		descLabels.push_back(Label("intensity", 1));
		assert(rosMsg.intensities.size() == rosMsg.ranges.size());
	}

	// filter points based on range
	std::vector <size_t> ids(rosMsg.ranges.size());
	std::vector<double> ranges(rosMsg.ranges.size());
	std::vector<double> intensities(rosMsg.intensities.size());

	size_t goodCount(0);
	for(size_t i = 0; i < rosMsg.ranges.size(); ++i)
	{
		const float range(rosMsg.ranges[i]);
		if(range >= rosMsg.range_min && range <= rosMsg.range_max)
		{
			ranges[goodCount] = range;
			ids[goodCount] = i;
			if(!rosMsg.intensities.empty())
			{
				intensities[goodCount] = rosMsg.intensities[i];
			}
			++goodCount;
		}
	}

	ids.resize(goodCount);
	ranges.resize(goodCount);
	if(!rosMsg.intensities.empty())
	{
		intensities.resize(goodCount);
	}

	DataPoints cloud(featLabels, descLabels, goodCount);
	cloud.getFeatureViewByName("pad").setConstant(1);

	// fill features
	for(size_t i = 0; i < ranges.size(); ++i)
	{
		const T angle = rosMsg.angle_min + ids[i] * rosMsg.angle_increment;
		const T range(ranges[i]);

		cloud.features(0, i) = cos(angle) * range;
		cloud.features(1, i) = sin(angle) * range;
	}

	// fill descriptors
	if(!rosMsg.intensities.empty())
	{
		auto is(cloud.getDescriptorViewByName("intensity"));
		for(size_t i = 0; i < intensities.size(); ++i)
		{
			is(0, i) = intensities[i];
		}
	}

	return cloud;
}

#endif