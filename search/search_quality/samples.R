max.distance <- 2e6
max.rank <- 255

name.scores <- c('Zero', 'Substring Prefix', 'Substring', 'Full Match Prefix', 'Full Match')
search.types <- c('POI', 'BUILDING', 'STREET', 'UNCLASSIFIED', 'VILLAGE', 'CITY', 'STATE', 'COUNTRY')
features <- c('DistanceToPivot', 'Rank', 'NameCoverage')
features <- append(features, name.scores[-1])
features <- append(features, search.types[-length(search.types)])

RelevanceToInt <- function(relevance) {
  switch(relevance, 'Irrelevant'=0, 'Relevant'=1, 'Vital'=3)
}

ComputeNDCG <- function(relevances) {
  dcg <- 0
  for (i in seq_along(relevances))
    dcg <- dcg + relevances[i] / log(1 + i, base=2)

  dcg.norm <- 0
  i <- 2
  for (relevance in sort(relevances, decreasing=TRUE)) {
    dcg.norm <- dcg.norm + relevance / log(i, base=2)
    i <- i + 1
  }

  ifelse(dcg.norm != 0, dcg / dcg.norm, 0)
}

ComputeNDCGsWithoutW <- function(data) {
  aggregate(Relevance ~ SampleId, data=data, ComputeNDCG)$Relevance
}

data <- read.csv('samples.csv')
data$DistanceToPivot <- min(data$DistanceToPivot, max.distance) / max.distance
data$Rank <- data$Rank / max.rank
data$Relevance <- sapply(as.character(data$Relevance), RelevanceToInt)

for (name.score in name.scores[-1])
  data[name.score] = as.numeric(as.character(data$NameScore) == name.score)

for (search.type in search.types[-length(search.types)])
  data[search.type] = as.numeric(as.character(data$SearchType) == search.type)

ndcgs <- ComputeNDCGsWithoutW(data)
print(sprintf('Current NDCG: %f, std: %f', mean(ndcgs), sd(ndcgs)))
