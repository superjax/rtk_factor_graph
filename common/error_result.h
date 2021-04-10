#include <variant>

#include "common/check.h"
#include "common/error.h"

#define RETURN_OR_ASSIGN(err_result) \
    ({                               \
        if (!err_result.ok())        \
        {                            \
            return err_result.err(); \
        }                            \
        err_result.res();            \
    })

namespace mc {
template <typename T>
class ErrorResult
{
    std::variant<Error, T> data_;

 private:
    ErrorResult();  // disable default constructor

 public:
    ErrorResult(const T& val) : data_(val) {}
    ErrorResult(const Error& err) : data_(err)
    {
        check(!err.ok(), "cannot form error result from Error::none()");
    }

    bool ok() const { return data_.index() == 1; }

    const T& res() const { return std::get<1>(data_); }
    const Error& err() const { return std::get<0>(data_); }
};

}  // namespace mc
