#include "Pointers.h"

#include <stdexcept>

namespace PLVS2
{
// Exercise declarations with incomplete types, as in Pointers.h.
class SharedProbe;
DECLARE_SMART_POINTERS(SharedProbe)
class RawProbe;
DECLARE_POINTERS(RawProbe)

class SharedProbe : public std::enable_shared_from_this<SharedProbe>
{
public:
    explicit SharedProbe(std::unique_ptr<int> value) : value(std::move(value)) {}
    ~SharedProbe() { ++destroyed; }
    std::unique_ptr<int> value;
    static int destroyed;
};
int SharedProbe::destroyed = 0;

class RawProbe
{
public:
    ~RawProbe() { ++destroyed; }
    static int destroyed;
};
int RawProbe::destroyed = 0;

DEFINE_SMART_POINTERS(SharedProbe)
DEFINE_POINTERS(RawProbe)
}

static void check(bool condition, const char* message)
{
    if(!condition)
        throw std::runtime_error(message);
}

int main()
{
    using namespace PLVS2;
    static_assert(std::is_same<MapPointPtr, MapPoint*>::value, "Keep raw declarations");
    static_assert(std::is_same<MapLinePtr, MapLine*>::value, "Keep raw declarations");
    static_assert(std::is_same<MapObjectPtr, MapObject*>::value, "Keep raw declarations");
    static_assert(std::is_same<KeyFramePtr, KeyFrame*>::value, "Keep raw declarations");
    static_assert(std::is_same<FramePtr, Frame*>::value, "Keep raw declarations");

    auto owner = SharedProbeNewPtr(std::unique_ptr<int>(new int(42)));
    check(*owner->value == 42, "Factory must forward move-only arguments");
    SharedProbeWeakPtr weak = owner;
    auto wrapped = WrapPtr(GetRawPtr(owner));
    auto wrappedAgain = WrapPtr(owner.get());
    check(wrapped.get() == owner.get(), "Wrapping must preserve object identity");
    check(!owner.owner_before(wrapped) && !wrapped.owner_before(owner),
          "Wrapping must reuse the owner's control block");
    check(!wrapped.owner_before(wrappedAgain) && !wrappedAgain.owner_before(wrapped),
          "Repeated wrapping must reuse the same control block");
    DeletePtr(owner);
    check(owner.use_count() == 3, "Shared DeletePtr must preserve its no-op contract");
    owner.reset();
    wrappedAgain.reset();
    check(!weak.expired() && SharedProbe::destroyed == 0,
          "Wrapped pointers must keep the object alive");
    wrapped.reset();
    check(weak.expired() && SharedProbe::destroyed == 1,
          "The last owner must delete the object exactly once");

    auto nullShared = WrapPtr(static_cast<SharedProbe*>(nullptr));
    check(!nullShared && nullShared.use_count() == 0, "Null wraps must be empty");
    SharedProbe unmanaged(std::unique_ptr<int>(new int(7)));
    bool rejected = false;
    try { WrapPtr(&unmanaged); }
    catch(const std::bad_weak_ptr&) { rejected = true; }
    check(rejected, "Unmanaged objects must not acquire a fabricated owner");
    check(unmanaged.weak_from_this().expired(), "Failed wrapping must not seed ownership");

    auto raw = RawProbeNewPtr();
    check(WrapPtr(raw) == raw && GetRawPtr(raw) == raw, "Raw helpers must preserve identity");
    DeletePtr(raw);
    check(RawProbe::destroyed == 1, "Raw deletion must still destroy the object");
    check(WrapPtr(static_cast<RawProbe*>(nullptr)) == nullptr, "Raw null wraps must work");
}
