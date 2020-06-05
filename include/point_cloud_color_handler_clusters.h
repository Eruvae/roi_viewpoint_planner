#ifndef POINT_CLOUD_COLOR_HANDLER_CLUSTERS_H
#define POINT_CLOUD_COLOR_HANDLER_CLUSTERS_H

#include <pcl/visualization/point_cloud_color_handlers.h>
#include <unordered_map>

namespace pcl
{
namespace visualization
{

template <typename PointT>
class PointCloudColorHandlerClusters : public PointCloudColorHandler<PointT>
{
  typedef typename PointCloudColorHandler<PointT>::PointCloud PointCloud;
  typedef typename PointCloud::Ptr PointCloudPtr;
  typedef typename PointCloud::ConstPtr PointCloudConstPtr;

  std::unordered_map<int, unsigned int> clusterMap;

  public:
    typedef boost::shared_ptr<PointCloudColorHandlerClusters<PointT> > Ptr;
    typedef boost::shared_ptr<const PointCloudColorHandlerClusters<PointT> > ConstPtr;

    /** \brief Constructor. */
    PointCloudColorHandlerClusters () :
      PointCloudColorHandler<PointT> ()
    {
      capable_ = true;
    }

    /** \brief Constructor. */
    PointCloudColorHandlerClusters (const PointCloudConstPtr &cloud, const std::vector<pcl::PointIndices> &clusters) :
      PointCloudColorHandler<PointT> (cloud)
    {
      capable_ = true;
      for (const pcl::PointIndices &cluster : clusters)
      {
        unsigned int cluster_id = 0;
        for (int index : cluster.indices)
        {
          clusterMap[index] = cluster_id;
        }
        cluster_id++;
      }
    }

    /** \brief Abstract getName method. */
    virtual std::string
    getName () const { return ("PointCloudColorHandlerClusters"); }

    /** \brief Get the name of the field used. */
    virtual std::string
    getFieldName () const { return ("[clusters]"); }

    /** \brief Obtain the actual color for the input dataset as vtk scalars.
      * \param[out] scalars the output scalars containing the color for the dataset
      * \return true if the operation was successful (the handler is capable and
      * the input cloud was given as a valid pointer), false otherwise
      */
    virtual bool
    getColor (vtkSmartPointer<vtkDataArray> &scalars) const
    {
      if (!capable_ || !cloud_)
        return (false);

      if (!scalars)
        scalars = vtkSmartPointer<vtkUnsignedCharArray>::New();
      scalars->SetNumberOfComponents (3);

      size_t nr_points = cloud_->points.size();
      reinterpret_cast<vtkUnsignedCharArray*>(&(*scalars))->SetNumberOfTuples(nr_points);

      unsigned char* colors = new unsigned char[nr_points * 3];

      // Color every point
      for (int cp = 0; cp < nr_points; ++cp)
      {
        // get color from lookup
        auto cluster_it = clusterMap.find(cp);
        if (cluster_it == clusterMap.end()) // point not in cluster, color white
        {
          colors[cp * 3 + 0] = static_cast<unsigned char> (255);
          colors[cp * 3 + 1] = static_cast<unsigned char> (255);
          colors[cp * 3 + 2] = static_cast<unsigned char> (255);
        }
        else
        {
          RGB color = pcl::GlasbeyLUT::at(cluster_it->second % pcl::GlasbeyLUT::size());
          colors[cp * 3 + 0] = static_cast<unsigned char> (color.r);
          colors[cp * 3 + 1] = static_cast<unsigned char> (color.g);
          colors[cp * 3 + 2] = static_cast<unsigned char> (color.b);
        }
      }
      reinterpret_cast<vtkUnsignedCharArray*>(&(*scalars))->SetArray (colors, 3 * nr_points, 0, vtkUnsignedCharArray::VTK_DATA_ARRAY_DELETE);
      return (true);
    }

  protected:
    // Members derived from the base class
    using PointCloudColorHandler<PointT>::cloud_;
    using PointCloudColorHandler<PointT>::capable_;
};

}
}

#endif // POINT_CLOUD_COLOR_HANDLER_CLUSTERS_H
