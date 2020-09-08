//
// Created by delphi on 05.11.19.
//

#ifndef MODROB_FIXED_VECTOR_HPP
#define MODROB_FIXED_VECTOR_HPP
#pragma once

#include <cstddef>
#include <type_traits>
#include <initializer_list>
#include <algorithm>

namespace modrob {
namespace system {
// originally by ThePhD
// see https://gist.github.com/ThePhD/8153067

template<typename T, std::size_t n, std::size_t a = std::alignment_of<T>::value>
class fixed_vector {
public:
    typedef T value_type;
    typedef T &reference;
    typedef const T &const_reference;
    typedef T *pointer_type;
    typedef std::size_t size_type;
    typedef std::ptrdiff_t difference_type;
    typedef pointer_type iterator;
    typedef const pointer_type const_iterator;

private:
    typename std::aligned_storage<sizeof(T) * n, a>::type items;
    std::size_t len;

    T *ptrat(std::size_t idx) {
        return static_cast<T *>( static_cast<void *>( &items )) + idx;
    }

    const T *ptrat(std::size_t idx) const {
        return static_cast<const T *>( static_cast<const void *>( &items )) + idx;
    }

    T &refat(std::size_t idx) {
        return *ptrat(idx);
    }

    const T &refat(std::size_t idx) const {
        return *ptrat(idx);
    }

public:
    constexpr static std::size_t max_size() {
        return n;
    }

    fixed_vector() : len(0) {

    }

    fixed_vector(std::size_t capacity) : len(std::min(n, capacity)) {

    }

    template<std::size_t c>
    fixed_vector(const T( &arr )[c]) : len(c) {
        static_assert(c < n, "Array too large to initialize fixed_vector");
        std::copy(std::addressof(arr[0]), std::addressof(arr[c]), data());
    }

    fixed_vector(std::initializer_list <T> initializer) : len(std::min(n, initializer.size())) {
        std::copy(initializer.begin(), initializer.begin() + len, data());
    }

    bool empty() const {
        return len < 1;
    }

    bool not_empty() const {
        return len > 0;
    }

    bool full() const {
        return len >= n;
    }

    void push_back(const T &item) {
        new(ptrat(len++)) T(item);
    }

    void push_back(T &&item) {
        new(ptrat(len++)) T(std::move(item));
    }

    template<typename ...Tn>
    reference emplace_back(Tn &&... argn) {
        new(ptrat(len++)) T(std::forward<Tn>(argn)...);
        return back();
    }

    void pop_back() {
        T &addr = refat(--len);
        addr.~T();
    }

    void clear() {
        for (; len > 0;) {
            pop_back();
        }
    }

    std::size_t size() const {
        return len;
    }

    std::size_t capacity() const {
        return n;
    }

    void resize(std::size_t sz) {
        len = std::min(sz, n);
    }

    T *data() {
        return ptrat(0);
    }

    const T *data() const {
        return ptrat(0);
    }

    T &operator[](std::size_t idx) {
        return refat(idx);
    }

    const T &operator[](std::size_t idx) const {
        return refat(idx);
    }

    T &front() {
        return refat(0);
    }

    T &back() {
        return refat(len - 1);
    }

    const T &front() const {
        return refat(0);
    }

    const T &back() const {
        return refat(len - 1);
    }

    T *begin() {
        return data();
    }

    const T *cbegin() {
        return data();
    }

    const T *begin() const {
        return data();
    }

    const T *cbegin() const {
        return data();
    }

    T *end() {
        return data() + len;
    }

    const T *cend() {
        return data() + len;
    }

    const T *end() const {
        return data() + len;
    }

    const T *cend() const {
        return data() + len;
    }
};

enum class buffer_on_full{
    drops_front,
    drops_value
};

template<
        typename T, std::size_t n,
        buffer_on_full on_full_policy=buffer_on_full::drops_front,
        std::size_t a = std::alignment_of<T>::value,
        typename=std::enable_if_t< (n>0) > >
class fixed_circular_buffer {
public:
    typedef T value_type;
    typedef T &reference;
    typedef const T &const_reference;
    typedef T *pointer_type;
    typedef std::size_t size_type;
    typedef std::ptrdiff_t difference_type;
    typedef pointer_type iterator;
    typedef const pointer_type const_iterator;

private:
    typename std::aligned_storage<sizeof(T) * n, a>::type items;
    std::size_t _len{0};
    std::size_t _front{0};
    std::size_t _back{0};

    T *ptrat(std::size_t idx) {
        return static_cast<T *>( static_cast<void *>( &items )) + idx;
    }

    const T *ptrat(std::size_t idx) const {
        return static_cast<const T *>( static_cast<const void *>( &items )) + idx;
    }

    T &refat(std::size_t idx) {
        return *ptrat(idx);
    }

    const T &refat(std::size_t idx) const {
        return *ptrat(idx);
    }
    size_t plus_mod_size(size_t i){
        i++;
        if(i >= n) i=0;
        return i;
    }
    size_t minus_mod_size(size_t i){
        i--;
        if(i >= n) i=n-1;
        return i;
    }

public:
    constexpr static std::size_t max_size() {
        return n;
    }

    fixed_circular_buffer(){
    }
    ~fixed_circular_buffer(){
        clear();
    }
    fixed_circular_buffer(const fixed_circular_buffer& ) = delete;
    fixed_circular_buffer(fixed_circular_buffer&& ) = delete;
    fixed_circular_buffer& operator=(const fixed_circular_buffer&)=delete;
    fixed_circular_buffer& operator=(fixed_circular_buffer&&)=delete;


    template<std::size_t c, typename=std::enable_if_t<(c <= n)> >
    fixed_circular_buffer(const T( &arr )[c]) : _len(c), _back(_len) {
        static_assert(c < n, "Array too large to initialize fixed_vector");
        std::copy(std::addressof(arr[0]), std::addressof(arr[c]), data());
    }

    fixed_circular_buffer(std::initializer_list <T> initializer) : _len(std::min(n, initializer.size())) {
        _back = _len - 1;
        std::copy(initializer.begin(), initializer.begin() + _len, data());
    }

    bool empty() const {
        return _len == 0;
    }

    bool not_empty() const {
        return _len > 0;
    }

    bool full() const {
        return _len >= n;
    }

    void push_back(const T &item) {
        if(full()) {
            if (on_full_policy == buffer_on_full::drops_front) {
                pop_front();
            } else return;
        }else{
            _len++;
        }
        new(ptrat(_back)) T(item);
        _back = plus_mod_size(_back);
    }

    void push_back(T &&item) {
        if(full()) {
            if (on_full_policy == buffer_on_full::drops_front) {
                pop_front();
            } else return;
        }else{
            _len++;
        }
        new(ptrat(_back)) T(std::move(item));
        _back = plus_mod_size(_back);
    }

    template<typename ...Tn >
    reference emplace_back(Tn &&... argn) {
        if(full()) {
            if(on_full_policy == buffer_on_full::drops_front) {
                pop_front();
            }else return back();
        }
        _len++;
        new(ptrat(_back)) T{std::forward<Tn>(argn)...};
        auto back = _back;
        _back = plus_mod_size(_back);
        return refat(back);
    }

    void pop_front() {
        if(empty()) return;
        T &addr = refat(_front);
        addr.~T();
        _front = plus_mod_size(_front);
        _len --;
    }

    void clear() {
        for (; _len > 0;) {
            pop_front();
        }
    }

    std::size_t size() const {
        return _len;
    }

    T *data() {
        return ptrat(0);
    }

    const T *data() const {
        return ptrat(0);
    }

    T &operator[](std::size_t idx) {
        return refat(idx);
    }

    const T &operator[](std::size_t idx) const {
        return refat(idx);
    }

    T &front() {
        return refat(_front);
    }

    const T &front() const {
        return refat(_front);
    }

    T &back() {
        return refat(minus_mod_size(_back));
    }

    const T &back() const {
        return refat(minus_mod_size(_back));
    }

};


} // namespace system
} // namespace modrob
#endif //MODROB_FIXED_VECTOR_HPP
