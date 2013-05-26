#ifndef PROGRAMOPTIONS_H
#define PROGRAMOPTIONS_H

#include <boost/property_tree/ptree.hpp>
#include <boost/any.hpp>
#include <boost/foreach.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

#include <string>

namespace KSRobot
{
namespace utils
{
#define DEFAULT_VAL  const boost::any& defVal = boost::any()

class ProgramOptions
{
public:
    typedef boost::shared_ptr<ProgramOptions>  Ptr;
    typedef boost::shared_ptr<const ProgramOptions> ConstPtr;
    
    class UserTypeInterface
    {
    public:
        virtual ~UserTypeInterface();
        virtual void Add(const std::string& name, ProgramOptions* po, const boost::any& value) = 0;
        virtual void Put(const std::string& name, ProgramOptions* po, const boost::any& value) = 0;
        virtual void Get(const std::string& name, ProgramOptions* po, boost::any& value) = 0;
    };
    
    
    ProgramOptions();
    ProgramOptions(const ProgramOptions& other);
    ~ProgramOptions();
    
    bool                                LoadFromFile(const std::string& filename);
    void                                SaveToFile(const std::string& filename);
    
    double                              GetDouble(const std::string& name, DEFAULT_VAL);
    void                                PutDouble(const std::string& name, double val, bool letMultiple = false);
    
    int                                 GetInt(const std::string& name, DEFAULT_VAL);
    void                                PutInt(const std::string& name, int val, bool letMultiple = false);
    
    std::string                         GetString(const std::string& name, DEFAULT_VAL);
    void                                PutString(const std::string& name, const std::string& val, bool letMultiple = false);
    
    bool                                GetBool(const std::string& name, DEFAULT_VAL);
    void                                PutBool(const std::string& name, bool val, bool letMultiple = false);
    
    bool                                NodeExists(const std::string& name) const;
    ProgramOptions::Ptr                 StartNode(const std::string& name);
    void                                operator = (const ProgramOptions& other);
    
    
    template<class X>
    X GetValue(const std::string& name)
    {
        MutexType::scoped_lock lock(*mGaurd);
        return mRoot->get<X>(mStrPrefix + name);
    }
    
    //NOTE: This also adds the current default value to the tree
    template<class X>
    X GetValue(const std::string& name, const X& defVal)
    {
        mGaurd->lock();
        boost::optional<X> ref = mRoot->get_optional<X>(mStrPrefix + name);
        mGaurd->unlock();
        if( !ref )
        {
            // add the default value.
            PutValue(name, defVal);
            return defVal;
        }
        return ref.get();
    }
    
    template<class X>
    void PutValue(const std::string& name, const X& val)
    {
        MutexType::scoped_lock lock(*mGaurd);
        mRoot->put<X>(mStrPrefix + name, val);
    }
    
    template<class X>
    void AddValue(const std::string& name, const X& val)
    {
        MutexType::scoped_lock lock(*mGaurd);
        mRoot->add<X>(mStrPrefix + name, val);
    }
    
private:
    typedef boost::property_tree::ptree TreeType;
    typedef boost::shared_ptr<TreeType> TreePtr;
    
    typedef boost::mutex                MutexType;
    
    TreePtr                             mRoot;
    std::string                         mStrPrefix;
    boost::shared_ptr<MutexType>        mGaurd;

    //TODO: Complete here
    typedef std::map<std::string, UserTypeInterface*>   UserTypesMap;
    static UserTypesMap                 mUserTypesMap;
    static boost::mutex                 mUserTypesGaurd;
};

#undef DEFAULT_VAL

} // end namespace utils
} // end namespace KSRobot
#endif // PROGRAMOPTIONS_H
