#ifndef PROGRAMOPTIONS_H
#define PROGRAMOPTIONS_H

#include <boost/property_tree/ptree.hpp>
#include <boost/any.hpp>
#include <boost/optional.hpp>
#include <boost/foreach.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <boost/type_traits.hpp>

#include <string>

namespace KSRobot
{
namespace utils
{

class ProgramOptions
{
public:
    typedef boost::shared_ptr<ProgramOptions>  Ptr;
    typedef boost::shared_ptr<const ProgramOptions> ConstPtr;

    template<class X>
    struct IsSupportedType
    {
        typedef boost::integral_constant<bool, ::boost::is_same<X, std::string>::value 
        || ::boost::is_integral<X>::value || ::boost::is_floating_point<X>::value > truth_type;
    };
    
    class UserTypeInterface
    {
    public:
        virtual ~UserTypeInterface() {;}
        virtual void            Put(ProgramOptions::Ptr po, const boost::any& value) = 0;
        virtual boost::any      Get(ProgramOptions::Ptr po) = 0;
    };
    
    
    ProgramOptions();
    ProgramOptions(const ProgramOptions& other);
    ~ProgramOptions();
    
    //TODO: Remove dependency on boost::any, and make them of boost::optional type.
    template<class X>
    static void                         AddUserType(UserTypeInterface* newType)
    {
        AddUserTypeInternal(typeid(X).name(), newType);
    }
    
    bool                                LoadFromFile(const std::string& filename);
    void                                SaveToFile(const std::string& filename);
    
    double                              GetDouble(const std::string& name, const boost::optional<double>& defVal = boost::none);
    void                                PutDouble(const std::string& name, double val);
    
    int                                 GetInt(const std::string& name, const boost::optional<int>& defVal = boost::none);
    void                                PutInt(const std::string& name, int val);
    
    std::string                         GetString(const std::string& name, const boost::optional<std::string>& defVal = boost::none);
    void                                PutString(const std::string& name, const std::string& val);
    
    bool                                GetBool(const std::string& name, const boost::optional<bool>& defVal = boost::none);
    void                                PutBool(const std::string& name, bool val);
    
    bool                                NodeExists(const std::string& name) const;
    ProgramOptions::Ptr                 StartNode(const std::string& name);
    void                                operator = (const ProgramOptions& other);

    void                                PutNodeValue(const std::string& val);
    std::string                         GetNodeValue();
    
    template<class X>
    X GetValue(const std::string& name, const boost::optional<X>& defVal = boost::none)
    {
        return GetValInternal<X>(name, defVal, typename IsSupportedType<X>::truth_type());
    }
    
    template<class X>
    void PutValue(const std::string& name, const X& val)
    {
        PutValInternal<X>(name, val, typename IsSupportedType<X>::truth_type());
    }

private:
    template<class X>
    X GetValInternal(const std::string& name, boost::optional<X> defVal, boost::true_type)
    {
        // should lock
        mGaurd->lock();
        boost::optional<X> ref = mRoot->get_optional<X>(mStrPrefix + name);
        mGaurd->unlock();
        
        if( !ref && !defVal )
        {
            throw std::runtime_error("(ProgramOptions::GetValInternal) Variable '" + mStrPrefix + name +
                            "' is not defined and no default value is provided.");
        }
        else if( !ref )
        {
            // write and return
            PutValue<X>(name, *defVal);
            return *defVal;
        }
        else
        {
            return *ref;
        }
    }
    
    template<class X>
    X GetValInternal(const std::string& name, boost::optional<X> defVal, boost::false_type)
    {
        UserTypeInterface* iface = FindInterface(typeid(X).name());
        assert(iface);
        if( NodeExists(name) )
            return boost::any_cast<X>(iface->Get(StartNode(name)));
        else
        {
            PutValue<X>(name, *defVal);
            return *defVal;
        }
    }
    
    template<class X>
    void PutValInternal(const std::string& name, const X& val, boost::true_type)
    {
        MutexType::scoped_lock lock(*mGaurd);
        mRoot->put<X>(mStrPrefix + name, val);
    }
    
    template<class X>
    void PutValInternal(const std::string& name, const X& val, boost::false_type)
    {
        //TODO: Implement this
        UserTypeInterface* iface = FindInterface(typeid(X).name());
        assert(iface);
        iface->Put(StartNode(name), boost::any(val));
    }
    
    static void AddUserTypeInternal(const std::string& name, UserTypeInterface* iface);
    static UserTypeInterface* FindInterface(const std::string& name);
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
    static MutexType                    mUserTypesGaurd;
};

#undef DEFAULT_VAL

} // end namespace utils
} // end namespace KSRobot
#endif // PROGRAMOPTIONS_H
