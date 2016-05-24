#include "kutility/progress_bar.h"
#include <ostream>

void progress_bar::reset()
{
   m_current = m_start;
   m_progress = 0;
   time(&m_starting_time);
}

void progress_bar::reset(int start, int end, int divisions)
{
   m_start = start;
   m_current = start;
   m_end = end;
   m_divisions = divisions;
   m_progress = 0;
   time(&m_starting_time);
}

std::ostream& progress_bar::operator>>(std::ostream& os) const
{
   if(m_current > (m_progress * (m_end - m_start) / m_divisions) || m_current == m_end)
   {
      ++m_progress;
      os << m_message << m_limit;
      for(int c = 1; c <= m_divisions; ++c)
      {
         if(c < m_progress || m_current == m_end) {
            os << m_done;
         }
         else if(c > m_progress) {
            os << m_notDone;
         }
         else {
            os << m_processing;
         }
      }
      os << m_limit;

      time_t now; time(&now);
      double percent = double(m_current-m_start)/double(m_end-m_start);
      double elapsed = difftime( now, m_starting_time );
      double eta = elapsed / percent;

      os<<" ";
      os.width(5);
      os.fill(' ');
      os.precision(3);
      os.setf( std::ios_base::right );
      os<<eta - elapsed;

      os<<" / ";
      os.width(5);
      os.fill(' ');
      os.precision(3);
      os.setf( std::ios_base::left  );
      os<< eta<<"  ";

      os << m_end_message;

      if(m_current == m_end) {
         os << "\n" << std::flush;
      }
      else {
         os << "  \r" << std::flush;
      }
   }

   return os;
}

const progress_bar& progress_bar::operator()(int current)
{
   m_current = current;
   return *this;
}

void progress_bar::set_text(const std::string& text)
{
   m_message = text;
}
void progress_bar::set_end_text( const std::string& text)
{
   m_end_message = text;
}

void progress_bar::set_format(const std::string& formatString)
{
   if(formatString.length() >= 4)
   {
      m_limit = formatString[0];
      m_done = formatString[1];
      m_processing = formatString[2];
      m_notDone = formatString[3];
   }
}
