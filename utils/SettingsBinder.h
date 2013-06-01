/*
 * <one line to give the library's name and an idea of what it does.>
 * Copyright (C) 2013  Kourosh <kourosh.sartipi@gmail.com>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */

#ifndef SETTINGSBINDER_H
#define SETTINGSBINDER_H

#include <utils/ProgramOptions.h>
#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <vector>

namespace KSRobot
{
namespace common
{

class SettingsBinder
{
public:
    SettingsBinder();
    ~SettingsBinder();

    void                        SetPO(ProgramOptions::Ptr po);
    
    void                        ReadSettings();
    void                        WriteSettings();
    
    template<class X>
    void                        AddVariableSetting(const std::string& name, X* varPtr, const X& defVal)
    {
        AddVariableSettingInternal<X>(name, varPtr, boost::optional<X>(defVal));
    }
    template<class X>
    void                        AddVariableSetting(const std::string& name, X* varPtr)
    {
        AddVariableSettingInternal<X>(name, varPtr, boost::none);
    }
    
    template <class X>
    void                        AddFunctionSetting(const std::string& name,
                                           typename boost::function<X ()> getter,
                                           typename boost::function<void (X)> setter, const X& defVal)
    {
        AddFunctionSettingInternal<X>(name, getter, setter, boost::optional<X>(defVal));
    }
    template <class X>
    void                        AddFunctionSetting(const std::string& name,
                                                   typename boost::function<X ()> getter, 
                                                   typename boost::function<void (X)> setter)
    {
        AddFunctionSettingInternal<X>(name, getter, setter, boost::none);
    }
private:
    template<class X>
    void                        AddVariableSettingInternal(const std::string& name, X* varPtr, 
                                                   const boost::optional<X>& defVal = boost::none);
    template <class X>
    void                        AddFunctionSettingInternal(const std::string& name,
                                                   typename boost::function<X ()> getter, typename boost::function<void (X)> setter,
                                                   const boost::optional<X>& defVal = boost::none);
    
    
    
    class GetSetBase
    {
    public:
        GetSetBase(const std::string& name) : mName(name) {;}
        virtual ~GetSetBase() {;}
        
        virtual void ReadFromFile(ProgramOptions::Ptr po) = 0;
        virtual void WriteToFile(ProgramOptions::Ptr po) = 0;
    protected:
        std::string  mName;
    };
    
    template <class X>
    class GSFunction : public GetSetBase
    {
    public:
        typedef boost::function<X()>                GetterFn;
        typedef boost::function<void (X)>           SetterFn;
        
        GSFunction(const std::string& name) : GetSetBase(name) {;}
        virtual ~GSFunction() {;}
        
        void SetFn(GetterFn getter, SetterFn setter) { mGetter = getter; mSetter = setter; }
        
        virtual void ReadFromFile(ProgramOptions::Ptr po)
        {
            mSetter(po->GetValue<X>(mName));
        }
        
        virtual void WriteToFile(ProgramOptions::Ptr po)
        {
            po->PutValue<X>(mName, mGetter());
        }
    protected:
        GetterFn            mGetter;
        SetterFn            mSetter;
    };
    
    template<class X>
    class GSVariable : public GetSetBase
    {
    public:
        GSVariable(const std::string& name, X* ptr) : GetSetBase(name), mPtr(ptr) {;}
        
        virtual void ReadFromFile(ProgramOptions::Ptr po)
        {
            *mPtr = po->GetValue<X>(mName);
        }
        
        virtual void WriteToFile(ProgramOptions::Ptr po)
        {
            po->PutValue<X>(mName, *mPtr);
        }
        
    protected:
        X*                  mPtr;
    };
    
    typedef std::vector<GetSetBase*>   DataArray;
    DataArray                   mData;
    ProgramOptions::Ptr         mPO;
};

// INLINE IMPLEMENTATION
template <class X>
void SettingsBinder::AddFunctionSettingInternal(const std::string& name,
                                        boost::function<X ()> getter,
                                        boost::function<void (X)> setter,
                                        const boost::optional<X>& defVal)
{
    GSFunction<X>* ptr = new GSFunction<X>(name);
    ptr->SetFn(getter, setter);
    mData.push_back(ptr);
    
    //Now if default value was passed, make sure it exists
    if( defVal && !mPO->NodeExists(name) )
        mPO->PutValue<X>(name, *defVal);
}

template<class X>
void SettingsBinder::AddVariableSettingInternal(const std::string& name, X* varPtr, const boost::optional<X>& defVal)
{
    GSVariable<X>* var = new GSVariable<X>(name, varPtr);
    mData.push_back(var);
    
    if( defVal && !mPO->NodeExists(name) )
        mPO->PutValue<X>(name, *defVal);
}


} // end namespace utils
} // end namespace KSRobot

#endif // SETTINGSBINDER_H
