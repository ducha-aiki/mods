#ifndef KUTILITY_PROGRESS_BAR_H
#define KUTILITY_PROGRESS_BAR_H

#include <string>
#include <ctime>

class progress_bar
{
public:
   explicit inline progress_bar(int start, int end, int divisions);

   void reset();

   void reset(int start, int end, int divisions);

   std::ostream& operator>>(std::ostream& os) const;

   const progress_bar& operator()(int current);

   void set_text(const std::string& text);

   void set_end_text( const std::string& text);

   void set_format(const std::string& formatString);

private:
   int m_start;
   int m_current;
   int m_end;
   int m_divisions;
   mutable int m_progress;
   time_t m_starting_time;

   std::string m_message;
   std::string m_end_message;
   std::string m_done;
   std::string m_processing;
   std::string m_notDone;
   std::string m_limit;
};


inline progress_bar::progress_bar(int start, int end, int divisions)
   : m_start(start),
     m_current(start),
     m_end(end),
     m_divisions(divisions),
     m_progress(0),
     m_message("Progress: "),
     m_end_message(" "),
     m_done("-"),
     m_processing(">"),
     m_notDone(" "),
     m_limit("|")
{
   time(&m_starting_time);
}

inline std::ostream& operator<<(std::ostream& os, const progress_bar& pb)
{
   return pb >> os;
}


#endif
