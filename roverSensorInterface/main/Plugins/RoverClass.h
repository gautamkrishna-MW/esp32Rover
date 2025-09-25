
#include "PluginClass.h"
#include <memory>

class Rover {
private:
    std::vector<std::shared_ptr<Plugin>> plugins;

public:
    void add_plugin(std::shared_ptr<Plugin> plugin) {
        plugins.push_back(plugin);
        plugin->init();
    }

    void start_all() {
        for (auto& p : plugins) p->start();
    }

    void stop_all() {
        for (auto& p : plugins) p->stop();
    }

    std::shared_ptr<Plugin> get_plugin(int id) {
        for (auto& p : plugins)
            if (p->id() == id) return p;
        return nullptr;
    }
};
