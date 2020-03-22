namespace mc {
namespace utils {

// Anything that inherits from this class cannot be copied or moved
class NonCopyable
{
 protected:
    NonCopyable() = default;
    ~NonCopyable() = default;

    NonCopyable(NonCopyable const &) = delete;
    void operator=(NonCopyable const &x) = delete;
};

}  // namespace utils
}  // namespace mc
