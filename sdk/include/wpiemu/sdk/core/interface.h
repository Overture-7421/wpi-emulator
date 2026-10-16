#pragma once

// Declares a pure-virtual interface type: public virtual destructor, protected special
// members (prevents slicing and accidental copies through the base). Usage, first line of body:
//
//   class IThing {
//       WPIEMU_INTERFACE(IThing)
//   public:
//       virtual void doIt() = 0;
//   };
#define WPIEMU_INTERFACE(Name)                       \
public:                                              \
    virtual ~Name() = default;                       \
                                                     \
protected:                                           \
    Name() = default;                                \
    Name(const Name&) = default;                     \
    Name& operator=(const Name&) = default;          \
    Name(Name&&) = default;                          \
    Name& operator=(Name&&) = default;               \
                                                     \
private:
