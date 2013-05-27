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

double ProgramOptions::GetDouble(const std::string& name, const boost::optional<double>& defVal)
{
    return GetValue<double>(name, defVal);
}

void ProgramOptions::PutDouble(const std::string& name, double val)
{
    PutValue<double>(name, val);
}

int ProgramOptions::GetInt(const std::string& name, const boost::optional<int>& defVal)
{
    return GetValue<int>(name, defVal);
}

void ProgramOptions::PutInt(const std::string& name, int val)
{
    PutValue<int>(name, val);
}

std::string ProgramOptions::GetString(const std::string& name, const boost::optional<std::string>& defVal)
{
    return GetValue<std::string>(name, defVal);
}

void ProgramOptions::PutString(const std::string& name, const std::string& val)
{
    PutValue<std::string>(name, val);
}

bool ProgramOptions::GetBool(const std::string& name, const boost::optional<bool>& defVal)
{
    return GetValue<bool>(name, defVal);
}

void ProgramOptions::PutBool(const std::string& name, bool val)
{
    PutValue<bool>(name, val);
}

bool ProgramOptions::NodeExists(const std::string& name) const
{
    boost::optional<boost::property_tree::ptree&> child = mRoot->get_child_optional(mStrPrefix + name);
    return (bool)child;
}

ProgramOptions::Ptr ProgramOptions::StartNode(const std::string& name)
{
    ProgramOptions::Ptr ptr(new ProgramOptions(*this));
    ptr->mStrPrefix += name + ".";
    return ptr;
}

void ProgramOptions::AddUserTypeInternal(const string& name, ProgramOptions::UserTypeInterface* iface)
{
    MutexType::scoped_lock lock(mUserTypesGaurd);
    UserTypesMap::iterator iter = mUserTypesMap.find(name);
    if( iter != mUserTypesMap.end() )
        throw std::runtime_error("(ProgramOptions::AddUserTypeInterna) Cannot add a type twice <typename=" + name + ">.");
    
    mUserTypesMap[name] = iface;
}

ProgramOptions::UserTypeInterface* ProgramOptions::FindInterface(const string& name)
{
    MutexType::scoped_lock lock(mUserTypesGaurd);
    UserTypesMap::iterator iter = mUserTypesMap.find(name);
    if( iter == mUserTypesMap.end() )
        throw std::runtime_error("(ProgramOptions::FindInterface) Unknown type was encountered <typename=" + name + ">.");
    return iter->second;
}

