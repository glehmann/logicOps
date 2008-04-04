#ifndef __itkNotEqualImageFilter_h
#define __itkNotEqualImageFilter_h

#include "itkBinaryFunctorImageFilter.h"
#include "itkNumericTraits.h"


namespace itk
{
  
/** \class NotEqualImageFilter
 * \brief Implements the NotEqual logical operator pixel-wise between two images.
 *
 * This class is parametrized over the types of the two 
 * input images and the type of the output image. 
 * Numeric conversions (castings) are done by the C++ defaults.
 *
 * 
 * The total operation over one pixel will be
 *
 *  output_pixel = static_cast<OutputPixelType>( input1_pixel != input2_pixel )
 *
 * Where "!=" is the equality operator in C++.
 *
 * \ingroup IntensityImageFilters  Multithreaded
 */
namespace Functor {  
  
template< class TInput1, class TInput2=TInput1, class TOutput=TInput1 >
class NotEqual
{
public:
  NotEqual() {};
  ~NotEqual() {};
  bool operator!=( const NotEqual & ) const
  {
    return false;
  }
  bool operator==( const NotEqual & other ) const
  {
    return !(*this != other);
  }
  inline TOutput operator()( const TInput1 & A, const TInput2 & B)
  {
    if( A != B )
      {
      return m_ForegroundValue;
      }
    return m_BackgroundValue;
  }

  TOutput m_ForegroundValue;
  TOutput m_BackgroundValue;
}; 

}
template <class TInputImage1, class TInputImage2, class TOutputImage>
class ITK_EXPORT NotEqualImageFilter :
    public
BinaryFunctorImageFilter<TInputImage1,TInputImage2,TOutputImage, 
                         Functor::NotEqual< 
  typename TInputImage1::PixelType, 
  typename TInputImage2::PixelType,
  typename TOutputImage::PixelType>   >


{
public:
  /** Standard class typedefs. */
  typedef NotEqualImageFilter  Self;
  typedef BinaryFunctorImageFilter<TInputImage1,TInputImage2,TOutputImage, 
                                   Functor::NotEqual< 
    typename TInputImage1::PixelType, 
    typename TInputImage2::PixelType,
    typename TOutputImage::PixelType>   
  >  Superclass;
  typedef SmartPointer<Self>   Pointer;
  typedef SmartPointer<const Self>  ConstPointer;

  /** Method for creation through the object factory. */
  itkNewMacro(Self);

  /** Runtime information support. */
  itkTypeMacro(NotEqualImageFilter,
               ImageToImageFilter);

  typedef typename TOutputImage::PixelType     OutputPixelType;

#ifdef ITK_USE_CONCEPT_CHECKING
  /** Begin concept checking */
  itkConceptMacro(OutputLogicalOperatorsCheck,
    (Concept::LogicalOperators<typename TOutputImage::PixelType>));
  /** End concept checking */
#endif

  /** Set the value in the image to consider as "foreground". Defaults to
   * maximum value of PixelType.*/
  itkSetMacro(ForegroundValue, OutputPixelType);

  /** Get the value in the image considered as "foreground". Defaults to
   * maximum value of PixelType. */
  itkGetConstMacro(ForegroundValue, OutputPixelType);

  /** Set the value used as "background". Defaults to
   * NumericTraits<PixelType>::NonpositiveMin(). */
  itkSetMacro(BackgroundValue, OutputPixelType);

  /** Get the value used as "background". Defaults to
   * NumericTraits<PixelType>::NonpositiveMin(). */
  itkGetConstMacro(BackgroundValue, OutputPixelType);
  

protected:
  NotEqualImageFilter()
    {
    m_ForegroundValue = NumericTraits<OutputPixelType>::max();
    m_BackgroundValue = NumericTraits<OutputPixelType>::NonpositiveMin();
    }
  virtual ~NotEqualImageFilter() {}

  void PrintSelf(std::ostream& os, Indent indent) const
    {
    Superclass::PrintSelf(os,indent);

    typedef typename NumericTraits<OutputPixelType>::PrintType
                                              OutputPixelPrintType;

    os << indent << "ForegroundValue: " 
                    << static_cast< OutputPixelPrintType > (m_ForegroundValue) 
                    << std::endl;
 
    os << indent << "BackgroundValue: " 
                    << static_cast< OutputPixelPrintType > (m_BackgroundValue) 
                    << std::endl;
    }

  void GenerateData()
    {
    this->GetFunctor().m_ForegroundValue = m_ForegroundValue;
    this->GetFunctor().m_BackgroundValue = m_BackgroundValue;
    Superclass::GenerateData();
    }

private:
  NotEqualImageFilter(const Self&); //purposely not implemented
  void operator=(const Self&); //purposely not implemented

  OutputPixelType m_ForegroundValue;
  OutputPixelType m_BackgroundValue;
  
};

} // end namespace itk


#endif
