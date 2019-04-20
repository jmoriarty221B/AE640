void imageCallback(const sensor_msgs::ImageConstPtr &image)
{
   long long sum = 0;
   for (int value : image->data)
   {
      sum += value;
   }
   int avg = sum / image->data.size();
   std::cout << "Brightness: " << avg << std::endl;
}

int main(int argc, char **argv)
{
   ros::init(argc, argv, "example_node");
   ros::NodeHandle n("~");
   ros::Subscriber sub = n.subscribe("/camera/rgb/image_raw", 10, imageCallback);
   ros::Rate loop_rate(50);
   while (ros::ok())
   {
      ros::spinOnce();
      loop_rate.sleep();
   }
}
