void sdetupRing(const SkeletonPtr& ring);
class MyWindow : public dart::gui::SimWindow
{
    public:

        MyWindow(const WorldPtr& world, const SkeletonPtr& ball,
                const SkeletonPtr& softBody, const SkeletonPtr& hybridBody,
                const SkeletonPtr& rigidChain, const SkeletonPtr& rigidRing);
        void keyboard(unsigned char key, int x, int y) override
            void drawWorld() const override;
        void displayTimer(int _val) override;
    protected:
        bool addObject(const SkeletonPtr& object);
        void addRing(const SkeletonPtr& ring);
        void removeSkeleton(const SkeletonPtr& skel);
        bool mRandomize;

        // std library objects that allow us to generate high-quality random numbers
        std::random_device mRD;
        std::mt19937 mMT;
        std::uniform_real_distribution<double> mDistribution;

        /// History of the active JointConstraints so that we can properly delete them
        /// when a Skeleton gets removed
        std::vector<dart::constraint::JointConstraintPtr> mJointConstraints;

        /// A blueprint Skeleton that we will use to spawn balls
        SkeletonPtr mOriginalBall;

        /// A blueprint Skeleton that we will use to spawn soft bodies
        SkeletonPtr mOriginalSoftBody;

        /// A blueprint Skeleton that we will use to spawn hybrid bodies
        SkeletonPtr mOriginalHybridBody;

        /// A blueprint Skeleton that we will use to spawn rigid chains
        SkeletonPtr mOriginalRigidChain;

        /// A blueprint Skeleton that we will use to spawn rigid rings
        SkeletonPtr mOriginalRigidRing;

        /// Keep track of how many Skeletons we spawn to ensure we can give them all
        /// unique names
        std::size_t mSkelCount;

};


template<class JointType>
BodyNode* addRigidBody(const SkeletonPtr& chain, const std::string& name,
        Shape::ShapeType type, BodyNode* parent = nullptr);
enum SoftShapeType {
    SOFT_BOX = 0,
    SOFT_CYLINDER,
    SOFT_ELLIPSOID
};

/// Add a soft body with the specified Joint type to a chain
template<class JointType>
BodyNode* addSoftBody(const SkeletonPtr& chain, const std::string& name,
        SoftShapeType type, BodyNode* parent = nullptr);
void setAllColors(const SkeletonPtr& object, const Eigen::Vector3d& color);
SkeletonPtr createBall();
SkeletonPtr createRigidChain();
SkeletonPtr createRigidRing();
SkeletonPtr createSoftBody();
SkeletonPtr createHybridBody();
SkeletonPtr createGround();
SkeletonPtr createWall();


int main();



