#pragma once

#include <memory>
#include <string>
#include <map>
#include <cstring>

#include <ArduinoJson.h>

#define CONFIGLET_NAME_SIZE 32

namespace Configlets
{
    using namespace std;

    class Configlet
    {
    public:
        Configlet(const char *name, const char *group = nullptr)
        {
            strncpy(_name, name, sizeof(_name) - 1);
            if (!group)
            {
                group = "default";
            }
            strncpy(_group, group, sizeof(_group) - 1);
        }

        virtual const char *dataType() = 0;

        void toJson(JsonObject &json)
        {
            json["name"] = name();
            json["group"] = group();

            toJsonInner(json);
        };

        const char *name() const { return _name; }
        const char *group() const { return _group; }

        bool readOnly = false;

    private:
        char _name[CONFIGLET_NAME_SIZE];
        char _group[CONFIGLET_NAME_SIZE];

        virtual void toJsonInner(JsonObject &json) = 0;
    };

    template <typename DataType>
    class CustomValue : public Configlet
    {
    public:
        CustomValue(const char *name, const char *group, shared_ptr<DataType> data)
            : Configlet(name, group), _data(data) {}
        const char *dataType() { return dtype(); }
        static const char *dtype() { return DataType::dtype(); }
        shared_ptr<DataType> value() { return _data; }

    private:
        shared_ptr<DataType> _data;

        void toJsonInner(JsonObject &json) { _data->toJson(json); }
    };

    class IntValue : public Configlet
    {
    private:
        int _value = 0;
        int _max_value = INT32_MAX;
        int _min_value = INT32_MIN;
        int _step = 1;

        void toJsonInner(JsonObject &json);

    public:
        IntValue(const char *name, const char *group = nullptr) : Configlet(name, group) {}
        const char *dataType() { return IntValue::dtype(); }
        static const char *dtype() { return "Int"; }

        int &value() { return _value; }
        int &max_value() { return _max_value; }
        int &min_value() { return _min_value; }
        int &step() { return _step; }
    };

    class FloatValue : public Configlet
    {
    private:
        shared_ptr<float> _pvalue;
        float _max_value = 999.;
        float _min_value = -999;
        float _step = 1.0;

        void toJsonInner(JsonObject &json);

    public:
        FloatValue(const char *name, const char *group, shared_ptr<float> pvalue)
            : Configlet(name, group), _pvalue(pvalue) {}

        const char *dataType() { return dtype(); }
        static const char *dtype() { return "Float"; }

        float &value() { return *_pvalue; }
        float &max_value() { return _max_value; }
        float &min_value() { return _min_value; }
        float &step() { return _step; }
    };

    class BoolValue : public Configlet
    {
    private:
        bool _value = false;

        void toJsonInner(JsonObject &json);

    public:
        BoolValue(const char *name, const char *group = nullptr) : Configlet(name, group) {}
        const char *dataType() { return dtype(); }
        static const char *dtype() { return "Bool"; }

        bool &value() { return _value; }
    };

    class StringValue : public Configlet
    {
    private:
        string _value;

        void toJsonInner(JsonObject &json);

    public:
        StringValue(const char *name, const char *group = nullptr) : Configlet(name, group) {}
        static const char *dtype() { return "String"; }
        const char *dataType() { return dtype(); }

        string &value() { return _value; }
    };

    class Registry
    {
    public:
        shared_ptr<IntValue> newIntValue(const char *name, const char *group = nullptr)
        {
            return newValue<IntValue>(name, group);
        }
        shared_ptr<FloatValue> newFloatValue(const char *name, const char *group = nullptr)
        {
            return newValue<FloatValue>(name, group, make_shared<float>(0.0));
        }
        shared_ptr<BoolValue> newBoolValue(const char *name, const char *group = nullptr)
        {
            return newValue<BoolValue>(name, group);
        }
        shared_ptr<StringValue> newStringValue(const char *name, const char *group = nullptr)
        {
            return newValue<StringValue>(name, group);
        }

        template <typename DataType, class... Args>
        shared_ptr<DataType> newValue(const char *name, const char *group, Args... _args)
        {
            static_assert(is_base_of<Configlet, DataType>::value, "DataType must derive Configlet");

            // xSemaphoreTake(xlock, portMAX_DELAY);

            {
                auto value = find(name, group);
                if (value)
                {
                    if (strcmp(value->dataType(), DataType::dtype()) != 0)
                        return nullptr;
                    return static_pointer_cast<DataType>(value);
                }
            }

            auto value = make_shared<DataType>(name, group, _args...);
            insert(value);

            // xSemaphoreGive(xlock);

            return value;
        }

        void dumpJson(JsonObject &json);

    private:
        std::map<string, std::map<string, shared_ptr<Configlet>>> registry;
        // SemaphoreHandle_t xlock = xSemaphoreCreateMutex();

        void insert(shared_ptr<Configlet> value);
        shared_ptr<Configlet> find(const char *name, const char *group = nullptr);
    };

    class VariableGroup
    {
    private:
        const char *group;
        Registry &reg;

    public:
        VariableGroup(const char *group, Registry &reg) : group(group), reg(reg) {}

        shared_ptr<IntValue> newIntValue(const char *name, int value = 0, int min_value = INT32_MIN, int max_value = INT32_MAX, int step = 1)
        {
            auto v = reg.newValue<IntValue>(name, group);
            v->value() = value;
            v->min_value() = min_value;
            v->max_value() = max_value;
            v->step() = step;
            return v;
        }
        shared_ptr<FloatValue> newFloatValue(const char *name, shared_ptr<float> pvalue, float min_value = -999., float max_value = 999, float step = 1.)
        {
            auto v = reg.newValue<FloatValue>(name, group, pvalue);
            v->min_value() = min_value;
            v->max_value() = max_value;
            v->step() = step;
            return v;
        }
        shared_ptr<BoolValue> newBoolValue(const char *name, bool value = false)
        {
            auto v = reg.newValue<BoolValue>(name, group);
            v->value() = value;
            return v;
        }
        shared_ptr<StringValue> newStringValue(const char *name, const char *value)
        {
            auto v = reg.newValue<StringValue>(name, group);
            v->value() = value;
            return v;
        }

        template <typename DataType>
        shared_ptr<CustomValue<DataType>> newCustomValue(const char *name, shared_ptr<DataType> pvalue)
        {
            return reg.newValue<CustomValue<DataType>>(name, group, pvalue);
        }
    };
};

using Configlet = Configlets::Configlet;
using Registry = Configlets::Registry;
