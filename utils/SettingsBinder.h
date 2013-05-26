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
namespace utils
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
    void                        AddVariableSetting(const std::string& name, X* varPtr);
    template <class X>
    void                        AddFunctionSetting(const std::string& name,
                                           boost::function<X ()> getter, boost::function<void (X)> setter);    
private:
    class GetSetBase
    {
    public:
        GetSetBase(ProgramOptions::Ptr po, const std::string& name) : mPO(po), mName(name) {;}
        virtual ~GetSetBase() {;}
        
        virtual void ReadFromFile() = 0;
        virtual void WriteToFile() = 0;
    protected:
        ProgramOptions::Ptr   mPO;
        std::string           mName;
    };
    
    template <class X>
    class GSFunction : public GetSetBase
    {
    public:
        typedef boost::function<X()>                GetterFn;
        typedef boost::function<void (X)>           SetterFn;
        
        GSFunction(ProgramOptions::Ptr po, const std::string& name) : GetSetBase(po, name) {;}
        virtual ~GSFunction() {;}
        
        void SetFn(GetterFn getter, SetterFn setter) { mGetter = getter; mSetter = setter; }
        
        virtual void ReadFromFile()
        {
            mSetter(mPO->GetValue<X>(mName));
        }
        
        virtual void WriteToFile()
        {
            mPO->PutValue<X>(mName, mGetter());
        }
    protected:
        GetterFn            mGetter;
        SetterFn            mSetter;
    };
    
    template<class X>
    class GSVariable : public GetSetBase
    {
    public:
        GSVariable(ProgramOptions::Ptr po, const std::string& name, X* ptr) : GetSetBase(po, name), mPtr(ptr) {;}
        
        virtual void ReadFromFile()
        {
            *mPtr = mPO->GetValue<X>(mName);
        }
        
        virtual void WriteToFile()
        {
            mPO->PutValue<X>(mName, *mPtr);
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
void SettingsBinder::AddFunctionSetting(const std::string& name,
                                               boost::function<X ()> getter, boost::function<void (X)> setter)
{
    GSFunction<X>* ptr = new GSFunction<X>(mPO, name);
    ptr->SetFn(getter, setter);
    mData.push_back(ptr);
}

template<class X>
void SettingsBinder::AddVariableSetting(const std::string& name, X* varPtr)
{
    //TODO: IMPLEMENT
    GSVariable<X>* var = new GSVariable<X>(mPO, name, varPtr);
    mData.push_back(var);
}



} // end namespace utils
} // end namespace KSRobot

#endif // SETTINGSBINDER_H
