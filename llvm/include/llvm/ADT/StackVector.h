//===- llvm/ADT/StackVector.h - Stack allocated vector ----------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
///
/// \file
/// This file defines the StackVector class.
///
//===----------------------------------------------------------------------===//

#ifndef LLVM_ADT_STACKVECTOR_H
#define LLVM_ADT_STACKVECTOR_H

#include "llvm/ADT/SmallVector.h"
#include <type_traits>
#include <algorithm>
#include <cassert>
#include <memory>
#include <limits>

namespace llvm {

template <typename T, unsigned N = CalculateSmallVectorDefaultInlinedElements<T>::value>
class StackVector {
  template <typename Iter>
  using EnableIfConvertibleToInputIterator =
    std::enable_if_t<
      std::is_convertible_v<typename std::iterator_traits<Iter>::iterator_category,
                            std::input_iterator_tag>,
    int>;

public:
  using value_type = T;
  using reference_type = T&;
  using const_reference_type = const T&;

  using reference = reference_type;
  using size_type = size_t;

  using iterator = T *;
  using const_iterator = const T *;
  using reverse_iterator = std::reverse_iterator<iterator>;
  using const_reverse_iterator = std::reverse_iterator<const_iterator>;

private:
  using NumElementsTy = unsigned char;

  static_assert(N <= 255, "Cannot store more than 255 elements!");

  union MaybeUninitialized {
    T Object;
    char Alias;
    constexpr MaybeUninitialized(): Alias{} {}
    ~MaybeUninitialized() {}
  };

  static_assert(sizeof(T) == sizeof(MaybeUninitialized) && alignof(T) == alignof(MaybeUninitialized),
                "T and MaybeUninitialized should have the exact same size!");

  MaybeUninitialized Elements[N] = {};
  NumElementsTy NumElements = 0;

  using ThisTy = StackVector<T, N>;

  constexpr void DestroyFrom(size_type Idx) {
    for (size_type I = Idx; I < NumElements; I++)
      Elements[I].Object.~T();
    NumElements = Idx;
  }

  template <bool ForOverwrite>
  constexpr void ResizeImpl(size_type Size) {
    assert(Size <= N && "Cannot resize beyond storage space!");

    if (Size <= NumElements)
      DestroyFrom(Size);
    else /* Size > NumElements */
      for (size_type I = NumElements; I < Size; I++) {
        if constexpr (ForOverwrite)
          new(&Elements[I].Object) T;
        else
          new(&Elements[I].Object) T{};
      }

    NumElements = Size;
  }

  // Copied from SmallVector.
  template <typename InputIterTy, typename OutputIterTy>
  constexpr void uninitialized_move(InputIterTy InputBegin, InputIterTy InputEnd, OutputIterTy OutputBegin) {
    if constexpr (std::is_trivially_copy_constructible_v<T> &&
                  std::is_trivially_move_constructible_v<T> &&
                  std::is_trivially_destructible_v<T>) {
      std::uninitialized_copy(InputBegin, InputEnd, OutputBegin);
    } else {
      std::uninitialized_move(InputBegin, InputEnd, OutputBegin);
    }
  }

  // Also copied from SmallVector
  constexpr bool MakeSpaceFor(iterator Iter, size_t NumToInsert) {
    iterator OldEnd = end();
    if (std::distance(Iter, end()) >= NumToInsert) {
      append(std::move_iterator<iterator>(end() - NumToInsert),
             std::move_iterator<iterator>(end()));
      std::move_backward(Iter, OldEnd - NumToInsert, OldEnd);
      return true;
    } else {
      size_type NumOverwritten = std::distance(Iter, OldEnd);
      NumElements += NumToInsert;
      uninitialized_move(Iter, OldEnd, std::next(OldEnd, (NumToInsert - NumOverwritten)));
      return false;
    }
  }

public:
  constexpr StackVector() = default;

  constexpr StackVector(const ThisTy &Other) : NumElements{Other.NumElements} {
    std::uninitialized_copy(Other.begin(), Other.end(), begin());
  }

  constexpr ThisTy &operator=(const ThisTy &Other) {
    DestroyFrom(0);
    std::uninitialized_copy(Other.begin(), Other.end(), begin());
    NumElements = Other.NumElements;
    return *this;
  }

  constexpr StackVector(ThisTy &&Other) : NumElements{Other.NumElements} {
    if (this == &Other)
      return;

    uninitialized_move(Other.begin(), Other.end(), begin());
    Other.NumElements = 0;
  }

  constexpr ThisTy &operator=(ThisTy &&Other) {
    if (this == &Other)
      return *this;

    if (NumElements > Other.NumElements) {
      iterator NewEnd = std::move(Other.begin(), Other.end(), begin());
      std::destroy(NewEnd, end());
    } else {
      std::move(Other.begin(), Other.begin() + NumElements, begin());
      uninitialized_move(Other.begin() + NumElements, Other.end(), begin() + NumElements);
    }
    NumElements = Other.NumElements;
    Other.clear();
    return *this;
  }

  constexpr StackVector(std::initializer_list<T> List) {
    assert(List.size() <= N && "Cannot have more initializers than space");
    for (size_type I = 0; I < List.size(); I++)
      new(&Elements[I].Object) T{std::data(List)[I]};
    NumElements = List.size();
  }

  template <typename U>
  constexpr StackVector(ArrayRef<U> Arr) {
    assert(Arr.size() <= N && "Cannot have more initializers than space available");
    append(Arr.begin(), Arr.end());
  }

  template <typename RangeTy>
  constexpr StackVector(const iterator_range<RangeTy> &Range) {
    assert(std::distance(Range.begin(), Range.end()) <= N && "Cannot have more initializers than space available!");
    append(Range.begin(), Range.end());
  }

  constexpr StackVector(size_type Size) {
    assert(Size <= N && "Cannot allocate more space than that available");
    resize(Size);
  }

  constexpr StackVector(size_type Size, const value_type &Value) {
    assert(Size <= N && "Cannot allocate more space than that available!");
    assign(Size, Value);
  }

  template <typename IterTy, EnableIfConvertibleToInputIterator<IterTy> = 0>
  StackVector(IterTy Begin, IterTy End) {
    append<IterTy>(Begin, End);
  }

  ~StackVector() {
    DestroyFrom(0);
  }

  constexpr iterator push_back(const T &Value) {
    assert(NumElements < N && "Cannot push into a full vector!");

    new(&Elements[NumElements++].Object) T{Value};
    return &Elements[NumElements - 1].Object;
  }

  constexpr iterator push_back(T &&Value) {
    assert(NumElements < N && "Cannot push into a full vector!");

    new(&Elements[NumElements++].Object) T{std::move(Value)};
    return &Elements[NumElements - 1].Object;
  }

  template <typename ...Ts>
  constexpr reference_type emplace_back(Ts &&...Args) {
    assert(NumElements < N && "Cannot emplace into full vector!");

    new(&Elements[NumElements++].Object) T{std::forward<Ts>(Args)...};
    return Elements[NumElements - 1].Object;
  }

  constexpr void pop_back() {
    assert(NumElements > 0 && "Cannot pop from empty StackVector!");
    Elements[--NumElements].Object.~T();
  }

  constexpr value_type pop_back_val() {
    T Ret = std::move(back());
    pop_back();
    return Ret;
  }

  constexpr void clear() {
    DestroyFrom(0);
  }

  [[nodiscard]] constexpr reference_type operator[](size_type Idx) {
    assert(Idx < NumElements && "Index must be in range!");
    return Elements[Idx].Object;
  }

  [[nodiscard]] constexpr const_reference_type operator[](size_type Idx) const {
    assert(Idx < NumElements && "Index must be in range!");
    return Elements[Idx].Object;
  }

  constexpr void resize(size_type Size) {
    ResizeImpl<false>(Size);
  }

  constexpr void resize_for_overwrite(size_type Size) {
    ResizeImpl<true>(Size);
  }

  constexpr void truncate(size_type Size) {
    assert(Size <= NumElements && "Cannot increase size with truncate!");
    ResizeImpl<false>(Size);
  }

  constexpr void resize(size_type Size, const value_type &Value) {
    if (Size < NumElements)
      truncate(Size);
    else
      append(Size - NumElements, Value);
  }

  constexpr void reserve(size_type Size) {
    assert(Size <= N && "Cannot make StackVector larger than stack space!");
    /* noop */
  }

  constexpr void pop_back_n(size_type NumItems) {
    assert(NumItems <= NumElements && "Cannot remove more elements than there are in the vector!");
    truncate(NumElements - NumItems);
  }

  constexpr void swap(ThisTy &Other) {
    for (size_type I = 0, E = std::min(NumElements, Other.NumElements); I < E; I++)
      std::swap(Elements[I].Object, Other.Elements[I].Object);

    // The case for == is already handled by the first for-loop
    if (NumElements > Other.NumElements)
      for (size_type I = Other.NumElements; I < NumElements; I++) {
        new(&Other.Elements[I].Object) T{std::move(Elements[I].Object)};
        Elements[I].Object.~T();
      }
    else if (Other.NumElements > NumElements)
      for (size_type I = NumElements; I < Other.NumElements; I++) {
        new(&Elements[I].Object) T{std::move(Other.Elements[I].Object)};
        Other.Elements[I].Object.~T();
      }

    std::swap(NumElements, Other.NumElements);
  }

  template <typename IterTy, EnableIfConvertibleToInputIterator<IterTy> = 0>
  constexpr void append(IterTy Begin, IterTy End) {
    size_type NumNewElements = std::distance(Begin, End);
    assert(NumElements + NumNewElements <= N && "Cannot append more elements than space available!");
    std::uninitialized_copy(Begin, End, end());
    NumElements += NumNewElements;
  }

  constexpr void append(size_type NumToInsert, const value_type &Value) {
    assert(NumElements + NumToInsert <= N && "Cannot append more elements than space available!");
    std::uninitialized_fill(end(), end() + NumToInsert, Value);
    NumElements += NumToInsert;
  }

  constexpr void append(std::initializer_list<T> List) {
    append(List.begin(), List.end());
  }

  constexpr void append(const ThisTy &Other) {
    assert(NumElements + Other.NumElements <= N && "Cannot append more elements than space available!");
    std::uninitialized_copy(Other.begin(), Other.end(), end());
    NumElements += Other.NumElements;
  }

  template <typename IterTy, EnableIfConvertibleToInputIterator<IterTy> = 0>
  constexpr void assign(IterTy Begin, IterTy End) {
    clear();
    append(Begin, End);
  }

  constexpr void assign(size_type NumToInsert, const value_type &Value) {
    assert(NumToInsert <= N && "Cannot assign more elements than space available!");

    std::fill(begin(), begin() + std::min(static_cast<size_type>(NumElements), NumToInsert), Value);
    if (NumToInsert < NumElements)
      truncate(NumToInsert);
    else if (NumToInsert > NumElements)
      std::uninitialized_fill(end(), end() + (NumToInsert - NumElements), Value);
    NumElements = NumToInsert;
  }

  iterator erase(const_iterator CIter) {
    auto Iter = const_cast<iterator>(CIter);
    std::move(Iter + 1, end(), Iter);
    pop_back();

    return Iter;
  }

  constexpr void assign(std::initializer_list<T> List) {
    clear();
    append(List);
  }

  constexpr void assign(const ThisTy &Other) {
    assign(Other.begin(), Other.end());
  }

  iterator erase(const_iterator CBegin, const_iterator CEnd) {
    auto Begin = const_cast<iterator>(CBegin);
    auto End = const_cast<iterator>(CEnd);
    size_type NumDeleted = std::distance(Begin, End);
    size_type NumDeletedStart = std::distance(begin(), Begin);
    size_type NumElementsLeftAtEnd = std::distance(End, end());

    for (size_type I = NumDeletedStart, E = NumDeletedStart + NumDeleted; I < E; I++)
      Elements[I].Object.~T();

    uninitialized_move(end() - NumElementsLeftAtEnd, end(), begin() + NumDeletedStart);
    NumElements -= NumDeleted;

    return Begin;
  }

  iterator insert(iterator Iter, value_type &&Value) {
    assert(NumElements < N && "Not enough space in vector to insert!");

    if (Iter == end()) {
      push_back(std::move(Value));
      return std::prev(end());
    }

    new(&Elements[NumElements].Object) T{std::move(back())};
    std::move_backward(Iter, end() - 1, end());
    value_type *ValuePtr = &Value;
    if (!std::less<>{}(ValuePtr, Iter) && std::less<>{}(ValuePtr, end())) 
      ValuePtr++;
    *Iter = std::move(*ValuePtr);
    NumElements++;

    return Iter;
  }

  iterator insert(iterator Iter, const value_type &Value) {
    assert(NumElements < N && "Not enough space in vector to insert!");

    if (Iter == end()) {
      push_back(Value);
      return std::prev(end());
    }

    new(&Elements[NumElements].Object) T{std::move(back())};
    std::move_backward(Iter, end() - 1, end());
    const value_type *ValuePtr = &Value;
    if (!std::less<>{}(ValuePtr, Iter) && std::less<>{}(ValuePtr, end())) 
      ValuePtr++;
    *Iter = *ValuePtr;
    NumElements++;

    return Iter;
  }

  iterator insert(iterator Iter, size_type NumToInsert, const value_type &Value) {
    assert(NumElements + NumToInsert <= N && "Not enough space in vector to insert!");

    if (Iter == end()) {
      append(NumToInsert, Value);
      return Iter;
    }

    iterator OldEnd = end();
    const value_type *ValuePtr = &Value;
    if (MakeSpaceFor(Iter, NumToInsert)) {
      if (!std::less<>{}(ValuePtr, Iter) && std::less<>{}(ValuePtr, end()))
        ValuePtr += NumToInsert;
      std::fill_n(Iter, NumToInsert, *ValuePtr);
    } else {
      size_type NumOverwritten = std::distance(Iter, OldEnd);
      if (!std::less<>{}(ValuePtr, Iter) && std::less<>{}(ValuePtr, end()))
        ValuePtr += NumToInsert;
      std::fill_n(Iter, NumOverwritten, *ValuePtr);
      std::uninitialized_fill_n(OldEnd, NumToInsert - NumOverwritten, *ValuePtr);
    }
    return Iter;
  }

  template <typename IterTy, EnableIfConvertibleToInputIterator<IterTy> = 0>
  iterator insert(iterator Iter, IterTy Begin, IterTy End) {
    size_type NumToInsert = std::distance(Begin, End);
    assert(NumElements + NumToInsert <= N && "Not enough space in vector to insert!");

    if (Iter == end()) {
      append(Begin, End);
      return Iter;
    }

    iterator OldEnd = end();
    if (MakeSpaceFor(Iter, NumToInsert)) {
      std::copy(Begin, End, Iter);
    } else {
      size_type NumOverwritten = std::distance(Iter, OldEnd);
      std::copy_n(Begin, NumOverwritten, Iter);
      std::uninitialized_copy(std::next(Begin, NumOverwritten), End, OldEnd);
    }
    return Iter;
  }

  [[nodiscard]] constexpr bool operator==(const ThisTy &Other) const {
    if (NumElements != Other.NumElements)
      return false;
    
    return std::equal(begin(), end(), Other.begin());
  }
  [[nodiscard]] constexpr bool operator!=(const ThisTy &Other) const { return !(*this == Other); }
  [[nodiscard]] constexpr bool operator<(const ThisTy &Other) const {
    return std::lexicographical_compare(this->begin(), this->end(),
                                        Other.begin(), Other.end());
  }
  [[nodiscard]] constexpr bool operator>(const ThisTy &Other)  const { return Other < *this; }
  [[nodiscard]] constexpr bool operator<=(const ThisTy &Other) const { return !(*this > Other); }
  [[nodiscard]] constexpr bool operator>=(const ThisTy &Other) const { return !(*this < Other); }

  [[nodiscard]] constexpr size_type size() const { return NumElements; }
  [[nodiscard]] constexpr size_type capacity() const { return N; }
  [[nodiscard]] constexpr bool empty() const { return NumElements == 0; }
  [[nodiscard]] constexpr T *data() { return &Elements[0].Object; }
  [[nodiscard]] constexpr const T *data() const { return &Elements[0].Object; }

  [[nodiscard]] constexpr reference_type front() { return Elements[0].Object; }
  [[nodiscard]] constexpr reference_type back()  { return Elements[NumElements - 1].Object; }
  [[nodiscard]] constexpr const_reference_type front() const { return Elements[0].Object; }
  [[nodiscard]] constexpr const_reference_type back()  const { return Elements[NumElements - 1].Object; }

  [[nodiscard]] constexpr iterator begin()              { return iterator{&Elements[0].Object}; }
  [[nodiscard]] constexpr iterator end()                { return iterator{&Elements[NumElements].Object}; }
  [[nodiscard]] constexpr const_iterator begin()  const { return const_iterator{&Elements[0].Object}; }
  [[nodiscard]] constexpr const_iterator cbegin() const { return const_iterator{&Elements[0].Object}; }
  [[nodiscard]] constexpr const_iterator end()    const { return const_iterator{&Elements[NumElements].Object}; }
  [[nodiscard]] constexpr const_iterator cend()   const { return const_iterator{&Elements[NumElements].Object}; }

  [[nodiscard]] constexpr reverse_iterator rbegin()              { return reverse_iterator{&Elements[NumElements].Object}; }
  [[nodiscard]] constexpr reverse_iterator rend()                { return reverse_iterator{&Elements[0].Object}; }
  [[nodiscard]] constexpr const_reverse_iterator rbegin()  const { return const_reverse_iterator{&Elements[NumElements].Object}; }
  [[nodiscard]] constexpr const_reverse_iterator crbegin() const { return const_reverse_iterator{&Elements[NumElements].Object}; }
  [[nodiscard]] constexpr const_reverse_iterator rend()    const { return const_reverse_iterator{&Elements[0].Object}; }
  [[nodiscard]] constexpr const_reverse_iterator crend()   const { return const_reverse_iterator{&Elements[0].Object}; }

  constexpr SmallVector<T, N> ToSmallVector() const {
    return SmallVector<T, N>(begin(), end());
  }

  constexpr SmallVector<T, N> MoveToSmallVector() {
    SmallVector<T, N> Ret;
    Ret.reserve(size());
    for (size_type Idx = 0; Idx < NumElements; Idx++)
      Ret.emplace_back(std::move(Elements[Idx].Object));
    NumElements = 0;
    return Ret;
  }
};

} // end namespace llvm

namespace std {
  /// Implement std::swap in terms of SmallVector swap.
  template <typename T, unsigned N>
  inline void
  swap(llvm::StackVector<T, N> &LHS, llvm::StackVector<T, N> &RHS) {
    LHS.swap(RHS);
  }
}

#endif
