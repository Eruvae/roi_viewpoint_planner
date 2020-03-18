// File not needed anymore, will be deleted
/*#ifndef SAMPLE_H
#define SAMPLE_H

#include <algorithm>
#include <type_traits>

template<typename _IntType, typename _UniformRandomBitGenerator>
std::pair<_IntType, _IntType> __gen_two_uniform_ints(_IntType __b0, _IntType __b1, _UniformRandomBitGenerator&& __g)
{
  _IntType __x = std::uniform_int_distribution<_IntType>{0, (__b0 * __b1) - 1}(__g);
  return std::make_pair(__x / __b1, __x % __b1);
}

/// Reservoir sampling algorithm.
template<typename _InputIterator, typename _RandomAccessIterator, typename _Size, typename _UniformRandomBitGenerator>
_RandomAccessIterator __sample(_InputIterator __first, _InputIterator __last, std::input_iterator_tag,
       _RandomAccessIterator __out, std::random_access_iterator_tag,
       _Size __n, _UniformRandomBitGenerator&& __g)
{
  using __distrib_type = std::uniform_int_distribution<_Size>;
  using __param_type = typename __distrib_type::param_type;
  __distrib_type __d{};
  _Size __sample_sz = 0;
  while (__first != __last && __sample_sz != __n)
  {
    __out[__sample_sz++] = *__first;
    ++__first;
  }
      for (auto __pop_sz = __sample_sz; __first != __last;
    ++__first, (void) ++__pop_sz)
  {
    const auto __k = __d(__g, __param_type{0, __pop_sz});
    if (__k < __n)
      __out[__k] = *__first;
  }
      return __out + __sample_sz;
}

/// Selection sampling algorithm.
template<typename _ForwardIterator, typename _OutputIterator, typename _Cat, typename _Size, typename _UniformRandomBitGenerator>
_OutputIterator __sample(_ForwardIterator __first, _ForwardIterator __last, std::forward_iterator_tag, _OutputIterator __out, _Cat, _Size __n, _UniformRandomBitGenerator&& __g)
{
  using __distrib_type = std::uniform_int_distribution<_Size>;
  using __param_type = typename __distrib_type::param_type;
  using _USize = std::make_unsigned<_Size>;
  using _Gen = std::remove_reference<_UniformRandomBitGenerator>;
  //using __uc_type = std::common_type<typename _UniformRandomBitGenerator::result_type, _USize>;

  __distrib_type __d{};
  _Size __unsampled_sz = std::distance(__first, __last);
  __n = std::min(__n, __unsampled_sz);

  // If possible, we use __gen_two_uniform_ints to efficiently produce two random numbers using a single distribution invocation:

  const size_t __urngrange = __g.max() - __g.min();
  if (__urngrange / size_t(__unsampled_sz) >= size_t(__unsampled_sz)) // I.e. (__urngrange >= __unsampled_sz * __unsampled_sz) but without wrapping issues.
  {
    while (__n != 0 && __unsampled_sz >= 2)
    {
      const std::pair<_Size, _Size> __p = __gen_two_uniform_ints(__unsampled_sz, __unsampled_sz - 1, __g);
      --__unsampled_sz;
      if (__p.first < __n)
      {
        *__out++ = *__first;
        --__n;
      }
      ++__first;

      if (__n == 0) break;

      --__unsampled_sz;
      if (__p.second < __n)
      {
        *__out++ = *__first;
        --__n;
      }
      ++__first;
    }
  }

  // The loop above is otherwise equivalent to this one-at-a-time version:
  for (; __n != 0; ++__first)
    if (__d(__g, __param_type{0, --__unsampled_sz}) < __n)
    {
      *__out++ = *__first;
      --__n;
    }

  return __out;
}

/// Take a random sample from a population.
template<typename _PopulationIterator, typename _SampleIterator, typename _Distance, typename _UniformRandomBitGenerator>
_SampleIterator sample(_PopulationIterator __first, _PopulationIterator __last, _SampleIterator __out, _Distance __n, _UniformRandomBitGenerator&& __g)
{
  using __pop_cat = typename std::iterator_traits<_PopulationIterator>::iterator_category;
  using __samp_cat = typename std::iterator_traits<_SampleIterator>::iterator_category;

  static_assert(std::__or_<std::is_convertible<__pop_cat, std::forward_iterator_tag>,
    std::is_convertible<__samp_cat, std::random_access_iterator_tag>>::value,
    "output range must use a RandomAccessIterator when input range does not meet the ForwardIterator requirements");

  static_assert(std::is_integral<_Distance>::value, "sample size must be an integer type");

  typename std::iterator_traits<_PopulationIterator>::difference_type __d = __n;
  return __sample(__first, __last, __pop_cat{}, __out, __samp_cat{}, __d, std::forward<_UniformRandomBitGenerator>(__g));
}

#endif // SAMPLE_H*/
