#include "Configlets.h"

namespace Configlets
{

    void Registry::insert(shared_ptr<Configlet> value)
    {
        auto sgroup = string(value->group());
        auto sname = string(value->name());
        registry[sgroup][sname] = value;
    }

    shared_ptr<Configlet> Registry::find(const char *name, const char *group)
    {
        auto sgroup = group ? string(group) : string("default");
        auto sname = string(name);
        auto cfs = registry.find(sgroup);
        if (cfs == registry.end())
        {
            return nullptr;
        }
        auto cf = cfs->second.find(sname);
        if (cf == cfs->second.end())
        {
            return nullptr;
        }
        return cf->second;
    }

    void Registry::dumpJson(JsonObject &json)
    {
        JsonArray vars = json.createNestedArray("vars");
        for (auto &cfs : registry)
        {
            for (auto &cf : cfs.second)
            {
                JsonObject var = vars.createNestedObject();
                cf.second->toJson(var);
            }
        }
    }

    void IntValue::toJsonInner(JsonObject &json)
    {
        JsonObject data = json.createNestedObject(dtype());
        data["value"] = _value;
        data["max_value"] = _max_value;
        data["min_value"] = _min_value;
        data["step"] = _step;
    }

    void FloatValue::toJsonInner(JsonObject &json)
    {
        JsonObject data = json.createNestedObject(dtype());
        data["value"] = value();
        data["max_value"] = _max_value;
        data["min_value"] = _min_value;
        data["step"] = _step;
    }

    void BoolValue::toJsonInner(JsonObject &json)
    {
        json[dtype()] = _value;
    }

    void StringValue::toJsonInner(JsonObject &json)
    {
        json[dtype()] = _value;
    }
};
