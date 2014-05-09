#include <mujinvision/visionmanagerimpl.h>

using namespace mujinvision;

class ColorImage
{
    ColorImage() {
    }
    virtual ~ColorImage() {
    }
};

class DepthImage
{
    DepthImage() {
    }
    virtual ~DepthImage() {
    }
};

class UserImageSubscriber : public ImageSubscriber
{
public:
    UserImageSubscriber() {
    }
    virtual ~UserImageSubscriber() {
    }

    void StartCaptureThread() {
    }

    void StopCaptureThread() {
    }

    void GetImagesFromMessage(std::vector<ImagePtr>& outimages) {
    }

    void GetImagesFromConsecutiveMessages(const unsigned int n, std::vector<ImagePtr>& outimages) {
    }

    void LoadParameters(const ptree& pt) {
    }

    void GetSpecificImages(const std::string& cameraid, ImageType imagetype, const unsigned int n, std::vector<ImagePtr>& outimages) {
    }
};
typedef boost::shared_ptr<UserImageSubscriber> UserImageSubscriberPtr;

class UserImageSubscriberManager : public ImageSubscriberManager
{
public:
    UserImageSubscriberManager(){
    }

    virtual ~UserImageSubscriberManager() {
    }

    void Initialize(const std::map<std::string, CameraPtr >&mNameCamera, const std::vector<ImageSubscriberPtr>&subscribers) {
    }

    ColorImagePtr GetColorImage(const std::string& cameraname, unsigned long long& timestamp)
    {
        return ColorImagePtr();
    }

    void GetConsecutiveColorImages(const std::string& cameraname, const unsigned int n, std::vector<ColorImagePtr>& colorimages, unsigned long long& starttime, unsigned long long& endtime) {
    }

    DepthImagePtr GetDepthImage(const std::string& cameraname, const unsigned int n, unsigned long long& starttime, unsigned long long& endtime)
    {
        return DepthImagePtr();
    }

    void WriteColorImage(ColorImageConstPtr colorimage, const std::string& filename) {
    }

    void WriteDepthImage(DepthImageConstPtr depthimage, const std::string& filename) {
    }

    ImageSubscriberPtr CreateImageSubscriber(const std::string& ip, const unsigned int port, const ptree& params_pt)
    {
        UserImageSubscriberPtr subscriber(new UserImageSubscriber());
        return boost::dynamic_pointer_cast<ImageSubscriber>(subscriber);
    }

};

typedef boost::shared_ptr<UserImageSubscriberManager> UserImageSubscriberManagerPtr;
typedef boost::shared_ptr<UserImageSubscriberManager const> UserImageSubscriberManagerConstPtr;
typedef boost::weak_ptr<UserImageSubscriberManager> UserImageSubscriberManagerWeakPtr;

class UserObjectDetector : public ObjectDetector
{
public:
    UserObjectDetector() {
    }

    virtual ~UserObjectDetector() {
    }

    void Initialize(const ptree& oparams_pt,  const ptree& dparams_pt, RegionConstPtr region, std::map<std::string, CameraPtr > mColorCamera, std::map<std::string, CameraPtr > mDepthCamera ) {
    }

    void DetectInColorImage(const std::string& colorcameraname, ColorImagePtr pcolorimage, std::vector<DetectedObjectPtr>& resultscolorcamera) {
    }

    void RefineDetectionWithDepthData(const std::string& colorcameraname, const std::string& depthcameraname, const std::vector<DetectedObjectPtr>& resultscolorcamera, std::vector<DetectedObjectPtr>& resultsdepthcamera, std::vector<unsigned int>& indicescolorcamera) {
    }

    void GetPointCloudObstacle(const std::string& depthcameraname, const std::vector<DetectedObjectPtr>& resultsdepthcamera, std::vector<double>& points, const double voxelsize=0.01) {
    }

    void GetCameraPointCloud(const std::string& depthcameraname, DepthImageConstPtr depthimage, std::vector<double>& points) {
    }

    void SetColorImage(const std::string& colorcameraname, ColorImageConstPtr colorimage, const unsigned int minu, const unsigned int maxu, const unsigned int minv, const unsigned int maxv) {
    }
};
typedef boost::shared_ptr<UserObjectDetector> UserObjectDetectorPtr;

class UserDetectorManager : public DetectorManager
{
public:
    UserDetectorManager() {
    }
    virtual ~UserDetectorManager() {
    }

    ObjectDetectorPtr CreateObjectDetector(const ptree& objectparams_pt, const ptree& detectionparams_pt, RegionConstPtr region, std::map<std::string, CameraPtr> mColorCamera, std::map<std::string, CameraPtr> mDepthCamera, const boost::function<void(const std::string& msg)>& setstatusfn)
    {
        UserObjectDetectorPtr detector(new UserObjectDetector());
        return boost::dynamic_pointer_cast<ObjectDetector>(detector);
    }
};

typedef boost::shared_ptr<UserDetectorManager> UserDetectorManagerPtr;
typedef boost::shared_ptr<UserDetectorManager const> UserDetectorManagerConstPtr;
typedef boost::weak_ptr<UserDetectorManager> UserDetectorManagerWeakPtr;

int main(int argc, char* argv[])
{
    const std::string visionmanagerConfigFilename = argv[1];
    UserImageSubscriberManagerPtr imagesubscribermanager(new UserImageSubscriberManager());
    UserDetectorManagerPtr detectormanager(new UserDetectorManager());
    MujinVisionManagerPtr visionmanager;
    visionmanager.reset(new MujinVisionManager(imagesubscribermanager, detectormanager, visionmanagerConfigFilename));
    while (!visionmanager->bShutdown) {
        boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
    }

}
