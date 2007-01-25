#ifndef __itkGreaterEqualImageFilter_h
#define __itkGreaterEqualImageFilter_h

#include "itkBinaryFunctorImageFilter.h"
#include "itkNumericTraits.h"


namespace itk
{
  
/** \class GreaterEqualImageFilter
 * \brief Implements the GreaterEqual logical operator pixel-wise between two images.
 *
 * This class is parametrized over the types of the two 
 * input images and the type of the output image. 
 * Numeric conversions (castings) are done by the C++ defaults.
 *
 * 
 * The total operation over one pixel will be
 *
 *  output_pixel = static_cast<OutputPixelType>( input1_pixel >= input2_pixel )
 *
 * Where ">=" is the greater than or equal operator in C++.
 *
 * \ingroup IntensityImageFilters  Multithreaded
 */
namespace Functor {  
  
template< class TInput1, class TInput2=TInput1, class TOutput=TInput1 >
class GREATEREQUAL
{
public:
  GREATEREQUAL() {};
  ~GREATEREQUAL() {};
  bool operator!=( const GREATEREQUAL & ) const
  {
    return false;
  }
  bool operator==( const GREATEREQUAL & other ) const
  {
    return !(*this != other);
  }
  inline TOutput operator()( const TInput1 & A, const TInput2 & B)
  {
    return static_cast<TOutput>( A >= B );
  }
}; 

}
template <class TInputImage1, class TInputImage2, class TOutputImage>
class ITK_EXPORT GreaterEqualImageFilter :
    public
BinaryFunctorImageFilter<TInputImage1,TInputImage2,TOutputImage, 
                         Functor::GREATEREQUAL< 
  typename TInputImage1::PixelType, 
  typename TInputImage2::PixelType,
  typename TOutputImage::PixelType>   >


{
public:
  /** Standard class typedefs. */
  typedef GreaterEqualImageFilter  Self;
  typedef BinaryFunctorImageFilter<TInputImage1,TInputImage2,TOutputImage, 
                                   Functor::GREATEREQUAL< 
    typename TInputImage1::PixelType, 
    typename TInputImage2::PixelType,
    typename TOutputImage::PixelType>   
  >  Superclass;
  typedef SmartPointer<Self>   Pointer;
  typedef SmartPointer<const Self>  ConstPointer;

  /** Method for creation through the object factory. */
  itkNewMacro(Self);

#ifdef ITK_USE_CONCEPT_CHECKING
  /** Begin concept checking */
  itkConceptMacro(Input1Input2OutputLogicalOperatorsCheck,
    (Concept::LogicalOperators<typename TInputImage1::PixelType,
                               typename TInputImage2::PixelType,
                               typename TOutputImage::PixelType>));
  /** End concept checking */
#endif

protected:
  GreaterEqualImageFilter() {}
  virtual ~GreaterEqualImageFilter() {}

private:
  GreaterEqualImageFilter(const Self&); //purposely not implemented
  void operator=(const Self&); //purposely not implemented

};

} // end namespace itk


#endif
