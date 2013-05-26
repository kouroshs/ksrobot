#include "ProgramOptions.h"
#include <boost/property_tree/xml_parser.hpp>
#include <boost/filesystem.hpp>
#include <boost/graph/graph_concepts.hpp>

#include <fstream>

using namespace KSRobot::utils;
namespace bt = boost::property_tree;

using namespace std;

ProgramOptions::UserTypesMap    ProgramOptions::mUserTypesMap;
boost::mutex                    ProgramOptions::mUserTypesGaurd;

ProgramOptions::ProgramOptions() : mRoot(new TreeType()), mStrPrefix(""), mGaurd(new MutexType())
{
}

ProgramOptions::ProgramOptions(const ProgramOptions& other) : mRoot(other.mRoot), 
        mStrPrefix(other.mStrPrefix)//, mGaurd(other.mGaurd)
{
    mGaurd = other.mGaurd;
}

ProgramOptions::~ProgramOptions()
{
}

void ProgramOptions::operator=(const ProgramOptions& other)
{
    mGaurd = other.mGaurd;
    mRoot = other.mRoot;
    mStrPrefix = other.mStrPrefix;
}

bool ProgramOptions::LoadFromFile(const std::string& filename)
{
    //first check if file exists, then check
    if( !boost::filesystem::exists(filename) )
    {
        std::cout << "(ProgramOptions::LoadFromFile) Configuration file does not exist.\n";
        return false;
    }
    try
    {
        MutexType::scoped_lock lock(*mGaurd);
        bt::read_xml(std::string(filename), *mRoot, boost::property_tree::xml_parser::trim_whitespace);
    }
    catch(std::exception& ex)
    {
        std::cerr << "(ProgramOptions::LoadFromFile) " << ex.what() << std::endl;
        return false;
    }
    return true;
}

void ProgramOptions::SaveToFile(const std::string& filename)
{
    try
    {
        MutexType::scoped_lock lock(*mGaurd);
        bt::xml_writer_settings<char> settings(' ', 4);
        bt::write_xml(std::string(filename), *mRoot, std::locale(), settings);
    }
    catch(std::exception& ex)
    {
        std::cerr << "(ProgramOptions::SaveToFile) " << ex.what() << std::endl;
    }
}

double ProgramOptions::GetDouble(const std::string& name, const boost::any& defVal)
{
    if( defVal.empty() )
        return GetValue<double>(name);
    else
        return GetValue<double>(name, boost::any_cast<double>(defVal));
}

void ProgramOptions::PutDouble(const std::string& name, double val, bool letMultiple)
{
    if( letMultiple )
        AddValue(name, val);
    else
        PutValue(name, val);
}

int ProgramOptions::GetInt(const std::string& name, const boost::any& defVal)
{
    if( defVal.empty() )
        return GetValue<int>(name);
    else
        return GetValue<int>(name, boost::any_cast<int>(defVal));
}

void ProgramOptions::PutInt(const std::string& name, int val, bool letMultiple)
{
    if( letMultiple )
        AddValue(name, val);
    else
        PutValue(name, val);
}

std::string ProgramOptions::GetString(const std::string& name, const boost::any& defVal)
{
    if( defVal.empty() )
        return GetValue<std::string>(name);
    else
        return GetValue<std::string>(name, boost::any_cast<std::string>(defVal));
}

void ProgramOptions::PutString(const std::string& name, const std::string& val, bool letMultiple)
{
    if( letMultiple )
        AddValue(name, val);
    else
        PutValue(name, val);
}

bool ProgramOptions::GetBool(const std::string& name, const boost::any& defVal)
{
    if( defVal.empty() )
        return GetValue<bool>(name);
    else
        return GetValue<bool>(name, boost::any_cast<bool>(defVal));
}

void ProgramOptions::PutBool(const std::string& name, bool val, bool letMultiple)
{
    if( letMultiple )
        AddValue(name, val);
    else
        PutValue(name, val);
}

bool ProgramOptions::NodeExists(const std::string& name) const
{
    boost::optional<boost::property_tree::ptree&> child = mRoot->get_child_optional(mStrPrefix + name);
    return (bool) child;
//     if( !child )
//         return false;
//     else
//         return true;
}

ProgramOptions::Ptr ProgramOptions::StartNode(const std::string& name)
{
    ProgramOptions::Ptr ptr(new ProgramOptions(*this));
    ptr->mStrPrefix += name + ".";
    return ptr;
//     std::cout << "before Loc\n" << std::flush;
//     boost::mutex::scoped_lock lock(*mGaurd);
//     std::cout << "after\n" << std::flush;
//     std::string otherPrefix = mStrPrefix + name + ".";
//     return ProgramOptions(this, otherPrefix);
}

template<class X>
class BaseTypeInterface : public ProgramOptions::UserTypeInterface
{
public:
    virtual ~BaseTypeInterface(){;}
    
    virtual void Add(const std::string& name, ProgramOptions* po, const boost::any& value)
    {
        po->AddValue<X>(name, boost::any_cast<X>(value));
    }
    
    virtual void Put(const std::string& name, ProgramOptions* po, const boost::any& value)
    {
        po->PutValue<X>(name, boost::any_cast<X>(value));
    }
    
    

};



