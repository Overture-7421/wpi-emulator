#pragma once

// Generic open/closed extension registry used by every "add X without touching core" seam
// (mechanism templates, device bridges, drivetrains, plant models, tire models, game plugins use
// dedicated wrappers around it).
//
//  * Instances, never globals: the composition root (app / test) owns each registry and passes it
//    to the modules that register into it.
//  * Registration order is preserved (deterministic UI lists / bridge start order).
//  * Registering a duplicate key is an error (AlreadyExists), never a silent override.
//  * Not thread-safe: populate during startup on one thread; afterwards it is read-only.

#include <functional>
#include <memory>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "wpiemu/sdk/core/result.h"

namespace wpiemu::sdk {

struct NoInfo {};

template <class Info, class Product, class... Args>
class Registry {
public:
    using Factory = std::function<std::unique_ptr<Product>(Args...)>;

    Status add(std::string key, Info info, Factory factory) {
        if (key.empty()) return makeError(ErrorCode::InvalidArgument, "registry key must not be empty");
        if (!factory) return makeError(ErrorCode::InvalidArgument, "registry factory must not be empty: " + key);
        if (contains(key)) return makeError(ErrorCode::AlreadyExists, "already registered: " + key);
        entries_.push_back(Entry{std::move(key), std::move(info), std::move(factory)});
        return okStatus();
    }

    bool contains(std::string_view key) const { return find(key) != nullptr; }
    std::size_t size() const { return entries_.size(); }
    bool empty() const { return entries_.empty(); }

    const Info* info(std::string_view key) const {
        const Entry* e = find(key);
        return e != nullptr ? &e->info : nullptr;
    }

    Result<std::unique_ptr<Product>> create(std::string_view key, Args... args) const {
        const Entry* e = find(key);
        if (e == nullptr) return makeError(ErrorCode::NotFound, "not registered: " + std::string(key));
        std::unique_ptr<Product> p = e->factory(std::forward<Args>(args)...);
        if (!p) return makeError(ErrorCode::Internal, "factory returned null: " + std::string(key));
        return p;
    }

    // Instantiates every entry in registration order; stops at the first failure. Only for factories without arguments.
    Result<std::vector<std::unique_ptr<Product>>> createAll() const
        requires(sizeof...(Args) == 0)
    {
        std::vector<std::unique_ptr<Product>> out;
        out.reserve(entries_.size());
        for (const Entry& e : entries_) {
            Result<std::unique_ptr<Product>> p = create(e.key);
            if (!p) return p.error();
            out.push_back(std::move(p).value());
        }
        return out;
    }

    std::vector<std::string> keys() const {
        std::vector<std::string> out;
        out.reserve(entries_.size());
        for (const auto& e : entries_) out.push_back(e.key);
        return out;
    }

    // fn(const std::string& key, const Info& info), in registration order.
    template <class Fn>
    void forEach(Fn&& fn) const {
        for (const auto& e : entries_) fn(e.key, e.info);
    }

private:
    struct Entry {
        std::string key;
        Info info;
        Factory factory;
    };
    const Entry* find(std::string_view key) const {
        for (const auto& e : entries_)
            if (e.key == key) return &e;
        return nullptr;
    }
    std::vector<Entry> entries_;
};

}  // namespace wpiemu::sdk
