#pragma once

#if PACKAGE_ROS_VERSION == 1
#else

#include "rclcpp/rclcpp.hpp"

class PublisherWrapperBase
{
   public:
	PublisherWrapperBase() = default;
	virtual ~PublisherWrapperBase() = default;
	virtual void publish(std::shared_ptr<void> message) = 0;
};

template <typename MessageT>
class PublisherWrapper : public PublisherWrapperBase
{
   public:
	PublisherWrapper(rclcpp::Node::SharedPtr node, const std::string& topic_name, size_t qos)
		: publisher_(node->create_publisher<MessageT>(topic_name, qos))
	{
	}

	void publish(std::shared_ptr<void> msg) override
	{
		const auto typed_msg = std::static_pointer_cast<MessageT>(msg);
		if (typed_msg)
		{
			publisher_->publish(*typed_msg);
		}
		else
		{
			RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to cast message to correct type.");
		}
	}

   private:
	typename rclcpp::Publisher<MessageT>::SharedPtr publisher_;
};

#endif
